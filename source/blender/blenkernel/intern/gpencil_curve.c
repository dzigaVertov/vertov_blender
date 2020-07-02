/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2008, Blender Foundation
 * This is a new part of Blender
 */

/** \file
 * \ingroup bke
 */

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CLG_log.h"

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_math_vector.h"

#include "BLT_translation.h"

#include "DNA_gpencil_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_collection.h"
#include "BKE_context.h"
#include "BKE_curve.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_curve.h"
#include "BKE_gpencil_geom.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_object.h"

#include "curve_fit_nd.h"

#include "DEG_depsgraph_query.h"

#define POINT_DIM 3

/* Helper: Check materials with same color. */
static int gpencil_check_same_material_color(Object *ob_gp, float color[4], Material **r_mat)
{
  Material *ma = NULL;
  float color_cu[4];
  linearrgb_to_srgb_v3_v3(color_cu, color);
  float hsv1[4];
  rgb_to_hsv_v(color_cu, hsv1);
  hsv1[3] = color[3];

  for (int i = 1; i <= ob_gp->totcol; i++) {
    ma = BKE_object_material_get(ob_gp, i);
    MaterialGPencilStyle *gp_style = ma->gp_style;
    /* Check color with small tolerance (better in HSV). */
    float hsv2[4];
    rgb_to_hsv_v(gp_style->fill_rgba, hsv2);
    hsv2[3] = gp_style->fill_rgba[3];
    if ((gp_style->fill_style == GP_MATERIAL_FILL_STYLE_SOLID) &&
        (compare_v4v4(hsv1, hsv2, 0.01f))) {
      *r_mat = ma;
      return i - 1;
    }
  }

  *r_mat = NULL;
  return -1;
}

/* Helper: Add gpencil material using curve material as base. */
static Material *gpencil_add_from_curve_material(Main *bmain,
                                                 Object *ob_gp,
                                                 const float cu_color[4],
                                                 const bool gpencil_lines,
                                                 const bool fill,
                                                 int *r_idx)
{
  Material *mat_gp = BKE_gpencil_object_material_new(
      bmain, ob_gp, (fill) ? "Material" : "Unassigned", r_idx);
  MaterialGPencilStyle *gp_style = mat_gp->gp_style;

  /* Stroke color. */
  if (gpencil_lines) {
    ARRAY_SET_ITEMS(gp_style->stroke_rgba, 0.0f, 0.0f, 0.0f, 1.0f);
    gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  }
  else {
    linearrgb_to_srgb_v4(gp_style->stroke_rgba, cu_color);
    gp_style->flag &= ~GP_MATERIAL_STROKE_SHOW;
  }

  /* Fill color. */
  linearrgb_to_srgb_v4(gp_style->fill_rgba, cu_color);
  /* Fill is false if the original curve hasn't material assigned, so enable it. */
  if (fill) {
    gp_style->flag |= GP_MATERIAL_FILL_SHOW;
  }

  /* Check at least one is enabled. */
  if (((gp_style->flag & GP_MATERIAL_STROKE_SHOW) == 0) &&
      ((gp_style->flag & GP_MATERIAL_FILL_SHOW) == 0)) {
    gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  }

  return mat_gp;
}

/* Helper: Create new stroke section. */
static void gpencil_add_new_points(bGPDstroke *gps,
                                   float *coord_array,
                                   float pressure,
                                   int init,
                                   int totpoints,
                                   const float init_co[3],
                                   bool last)
{
  for (int i = 0; i < totpoints; i++) {
    bGPDspoint *pt = &gps->points[i + init];
    copy_v3_v3(&pt->x, &coord_array[3 * i]);
    /* Be sure the last point is not on top of the first point of the curve or
     * the close of the stroke will produce glitches. */
    if ((last) && (i > 0) && (i == totpoints - 1)) {
      float dist = len_v3v3(init_co, &pt->x);
      if (dist < 0.1f) {
        /* Interpolate between previous point and current to back slightly. */
        bGPDspoint *pt_prev = &gps->points[i + init - 1];
        interp_v3_v3v3(&pt->x, &pt_prev->x, &pt->x, 0.95f);
      }
    }

    pt->pressure = pressure;
    pt->strength = 1.0f;
  }
}

/* Helper: Get the first collection that includes the object. */
static Collection *gpencil_get_parent_collection(Scene *scene, Object *ob)
{
  Collection *mycol = NULL;
  FOREACH_SCENE_COLLECTION_BEGIN (scene, collection) {
    LISTBASE_FOREACH (CollectionObject *, cob, &collection->gobject) {
      if ((mycol == NULL) && (cob->ob == ob)) {
        mycol = collection;
      }
    }
  }
  FOREACH_SCENE_COLLECTION_END;

  return mycol;
}

/* Helper: Convert one spline to grease pencil stroke. */
static void gpencil_convert_spline(Main *bmain,
                                   Object *ob_gp,
                                   Object *ob_cu,
                                   const bool gpencil_lines,
                                   const bool only_stroke,
                                   bGPDframe *gpf,
                                   Nurb *nu)
{
  Curve *cu = (Curve *)ob_cu->data;
  bGPdata *gpd = (bGPdata *)ob_gp->data;
  bool cyclic = true;

  /* Create Stroke. */
  bGPDstroke *gps = MEM_callocN(sizeof(bGPDstroke), "bGPDstroke");
  gps->thickness = 10.0f;
  gps->fill_opacity_fac = 1.0f;
  gps->hardeness = 1.0f;
  gps->uv_scale = 1.0f;

  ARRAY_SET_ITEMS(gps->aspect_ratio, 1.0f, 1.0f);
  ARRAY_SET_ITEMS(gps->caps, GP_STROKE_CAP_ROUND, GP_STROKE_CAP_ROUND);
  gps->inittime = 0.0f;

  gps->flag &= ~GP_STROKE_SELECT;
  gps->flag |= GP_STROKE_3DSPACE;

  gps->mat_nr = 0;
  /* Count total points
   * The total of points must consider that last point of each segment is equal to the first
   * point of next segment.
   */
  int totpoints = 0;
  int segments = 0;
  int resolu = nu->resolu + 1;
  segments = nu->pntsu;
  if ((nu->flagu & CU_NURB_CYCLIC) == 0) {
    segments--;
    cyclic = false;
  }
  totpoints = (resolu * segments) - (segments - 1);

  /* Materials
   * Notice: The color of the material is the color of viewport and not the final shader color.
   */
  Material *mat_gp = NULL;
  bool fill = true;
  /* Check if grease pencil has a material with same color.*/
  float color[4];
  if ((cu->mat) && (*cu->mat)) {
    Material *mat_cu = *cu->mat;
    copy_v4_v4(color, &mat_cu->r);
  }
  else {
    /* Gray (unassigned from SVG add-on) */
    zero_v4(color);
    add_v3_fl(color, 0.6f);
    color[3] = 1.0f;
    fill = false;
  }

  /* Special case: If the color was created by the SVG add-on and the name contains '_stroke' and
   * there is only one color, the stroke must not be closed, fill to false and use for
   * stroke the fill color.
   */
  bool do_stroke = false;
  if (ob_cu->totcol == 1) {
    Material *ma_stroke = BKE_object_material_get(ob_cu, 1);
    if ((ma_stroke) && (strstr(ma_stroke->id.name, "_stroke") != NULL)) {
      do_stroke = true;
    }
  }

  int r_idx = gpencil_check_same_material_color(ob_gp, color, &mat_gp);
  if ((ob_cu->totcol > 0) && (r_idx < 0)) {
    Material *mat_curve = BKE_object_material_get(ob_cu, 1);
    mat_gp = gpencil_add_from_curve_material(bmain, ob_gp, color, gpencil_lines, fill, &r_idx);

    if ((mat_curve) && (mat_curve->gp_style != NULL)) {
      MaterialGPencilStyle *gp_style_cur = mat_curve->gp_style;
      MaterialGPencilStyle *gp_style_gp = mat_gp->gp_style;

      copy_v4_v4(gp_style_gp->mix_rgba, gp_style_cur->mix_rgba);
      gp_style_gp->fill_style = gp_style_cur->fill_style;
      gp_style_gp->mix_factor = gp_style_cur->mix_factor;
    }

    /* If object has more than 1 material, use second material for stroke color. */
    if ((!only_stroke) && (ob_cu->totcol > 1) && (BKE_object_material_get(ob_cu, 2))) {
      mat_curve = BKE_object_material_get(ob_cu, 2);
      if (mat_curve) {
        linearrgb_to_srgb_v3_v3(mat_gp->gp_style->stroke_rgba, &mat_curve->r);
        mat_gp->gp_style->stroke_rgba[3] = mat_curve->a;
      }
    }
    else if ((only_stroke) || (do_stroke)) {
      /* Also use the first color if the fill is none for stroke color. */
      if (ob_cu->totcol > 0) {
        mat_curve = BKE_object_material_get(ob_cu, 1);
        if (mat_curve) {
          copy_v3_v3(mat_gp->gp_style->stroke_rgba, &mat_curve->r);
          mat_gp->gp_style->stroke_rgba[3] = mat_curve->a;
          /* Set fill and stroke depending of curve type (3D or 2D). */
          if ((cu->flag & CU_3D) || ((cu->flag & (CU_FRONT | CU_BACK)) == 0)) {
            mat_gp->gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
            mat_gp->gp_style->flag &= ~GP_MATERIAL_FILL_SHOW;
          }
          else {
            mat_gp->gp_style->flag &= ~GP_MATERIAL_STROKE_SHOW;
            mat_gp->gp_style->flag |= GP_MATERIAL_FILL_SHOW;
          }
        }
      }
    }
  }
  CLAMP_MIN(r_idx, 0);

  /* Assign material index to stroke. */
  gps->mat_nr = r_idx;

  /* Add stroke to frame.*/
  BLI_addtail(&gpf->strokes, gps);

  float *coord_array = NULL;
  float init_co[3];

  switch (nu->type) {
    case CU_POLY: {
      /* Allocate memory for storage points. */
      gps->totpoints = nu->pntsu;
      gps->points = MEM_callocN(sizeof(bGPDspoint) * gps->totpoints, "gp_stroke_points");
      /* Increase thickness for this type. */
      gps->thickness = 10.0f;

      /* Get all curve points */
      for (int s = 0; s < gps->totpoints; s++) {
        BPoint *bp = &nu->bp[s];
        bGPDspoint *pt = &gps->points[s];
        copy_v3_v3(&pt->x, bp->vec);
        pt->pressure = bp->radius;
        pt->strength = 1.0f;
      }
      break;
    }
    case CU_BEZIER: {
      /* Allocate memory for storage points. */
      gps->totpoints = totpoints;
      gps->points = MEM_callocN(sizeof(bGPDspoint) * gps->totpoints, "gp_stroke_points");

      int init = 0;
      resolu = nu->resolu + 1;
      segments = nu->pntsu;
      if ((nu->flagu & CU_NURB_CYCLIC) == 0) {
        segments--;
      }
      /* Get all interpolated curve points of Beziert */
      for (int s = 0; s < segments; s++) {
        int inext = (s + 1) % nu->pntsu;
        BezTriple *prevbezt = &nu->bezt[s];
        BezTriple *bezt = &nu->bezt[inext];
        bool last = (bool)(s == segments - 1);

        coord_array = MEM_callocN((size_t)3 * resolu * sizeof(float), __func__);

        for (int j = 0; j < 3; j++) {
          BKE_curve_forward_diff_bezier(prevbezt->vec[1][j],
                                        prevbezt->vec[2][j],
                                        bezt->vec[0][j],
                                        bezt->vec[1][j],
                                        coord_array + j,
                                        resolu - 1,
                                        3 * sizeof(float));
        }
        /* Save first point coordinates. */
        if (s == 0) {
          copy_v3_v3(init_co, &coord_array[0]);
        }
        /* Add points to the stroke */
        gpencil_add_new_points(gps, coord_array, bezt->radius, init, resolu, init_co, last);
        /* Free memory. */
        MEM_SAFE_FREE(coord_array);

        /* As the last point of segment is the first point of next segment, back one array
         * element to avoid duplicated points on the same location.
         */
        init += resolu - 1;
      }
      break;
    }
    case CU_NURBS: {
      if (nu->pntsv == 1) {

        int nurb_points;
        if (nu->flagu & CU_NURB_CYCLIC) {
          resolu++;
          nurb_points = nu->pntsu * resolu;
        }
        else {
          nurb_points = (nu->pntsu - 1) * resolu;
        }
        /* Get all curve points. */
        coord_array = MEM_callocN(sizeof(float[3]) * nurb_points, __func__);
        BKE_nurb_makeCurve(nu, coord_array, NULL, NULL, NULL, resolu, sizeof(float[3]));

        /* Allocate memory for storage points. */
        gps->totpoints = nurb_points - 1;
        gps->points = MEM_callocN(sizeof(bGPDspoint) * gps->totpoints, "gp_stroke_points");

        /* Add points. */
        gpencil_add_new_points(gps, coord_array, 1.0f, 0, gps->totpoints, init_co, false);

        MEM_SAFE_FREE(coord_array);
      }
      break;
    }
    default: {
      break;
    }
  }
  /* Cyclic curve, close stroke. */
  if ((cyclic) && (!do_stroke)) {
    BKE_gpencil_stroke_close(gps);
  }

  /* Recalc fill geometry. */
  BKE_gpencil_stroke_geometry_update(gpd, gps);
}

/**
 * Convert a curve object to grease pencil stroke.
 *
 * \param bmain: Main thread pointer
 * \param scene: Original scene.
 * \param ob_gp: Grease pencil object to add strokes.
 * \param ob_cu: Curve to convert.
 * \param gpencil_lines: Use lines for strokes.
 * \param use_collections: Create layers using collection names.
 * \param only_stroke: The material must be only stroke without fill.
 */
void BKE_gpencil_convert_curve(Main *bmain,
                               Scene *scene,
                               Object *ob_gp,
                               Object *ob_cu,
                               const bool gpencil_lines,
                               const bool use_collections,
                               const bool only_stroke)
{
  if (ELEM(NULL, ob_gp, ob_cu) || (ob_gp->type != OB_GPENCIL) || (ob_gp->data == NULL)) {
    return;
  }

  Curve *cu = (Curve *)ob_cu->data;
  bGPdata *gpd = (bGPdata *)ob_gp->data;
  bGPDlayer *gpl = NULL;

  /* If the curve is empty, cancel. */
  if (cu->nurb.first == NULL) {
    return;
  }

  /* Check if there is an active layer. */
  if (use_collections) {
    Collection *collection = gpencil_get_parent_collection(scene, ob_cu);
    if (collection != NULL) {
      gpl = BKE_gpencil_layer_named_get(gpd, collection->id.name + 2);
      if (gpl == NULL) {
        gpl = BKE_gpencil_layer_addnew(gpd, collection->id.name + 2, true);
      }
    }
  }

  if (gpl == NULL) {
    gpl = BKE_gpencil_layer_active_get(gpd);
    if (gpl == NULL) {
      gpl = BKE_gpencil_layer_addnew(gpd, DATA_("GP_Layer"), true);
    }
  }

  /* Check if there is an active frame and add if needed. */
  bGPDframe *gpf = BKE_gpencil_layer_frame_get(gpl, CFRA, GP_GETFRAME_ADD_COPY);

  /* Read all splines of the curve and create a stroke for each. */
  LISTBASE_FOREACH (Nurb *, nu, &cu->nurb) {
    gpencil_convert_spline(bmain, ob_gp, ob_cu, gpencil_lines, only_stroke, gpf, nu);
  }

  /* Tag for recalculation */
  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY | ID_RECALC_COPY_ON_WRITE);
}

/**
 * Creates a bGPDcurve by doing a cubic curve fitting on the grease pencil stroke points.
 */
bGPDcurve *BKE_gpencil_stroke_editcurve_generate(bGPDstroke *gps, float error_threshold)
{
  if (gps->totpoints < 1) {
    return NULL;
  }

  float *points = MEM_callocN(sizeof(float) * gps->totpoints * POINT_DIM, __func__);
  for (int i = 0; i < gps->totpoints; i++) {
    bGPDspoint *pt = &gps->points[i];
    float *to = &points[i * POINT_DIM];
    copy_v3_v3(to, &pt->x);
  }

  float *r_cubic_array = NULL;
  unsigned int r_cubic_array_len = 0;
  unsigned int *r_cubic_orig_index = NULL;
  unsigned int *r_corners_index_array = NULL;
  unsigned int r_corners_index_len = 0;
  int r = curve_fit_cubic_to_points_fl(points,
                                       gps->totpoints,
                                       POINT_DIM,
                                       error_threshold,
                                       CURVE_FIT_CALC_HIGH_QUALIY,
                                       NULL,
                                       0,
                                       &r_cubic_array,
                                       &r_cubic_array_len,
                                       &r_cubic_orig_index,
                                       &r_corners_index_array,
                                       &r_corners_index_len);

  if (r != 0 || r_cubic_array_len < 1) {
    return NULL;
  }

  bGPDcurve *editcurve = BKE_gpencil_stroke_editcurve_new(r_cubic_array_len);

  for (int i = 0; i < r_cubic_array_len; i++) {
    bGPDcurve_point *cpt = &editcurve->curve_points[i];
    BezTriple *bezt = &cpt->bezt;
    bGPDspoint *orig_pt = &gps->points[r_cubic_orig_index[i]];
    for (int j = 0; j < 3; j++) {
      copy_v3_v3(bezt->vec[j], &r_cubic_array[i * 3 * POINT_DIM + j * 3]);
    }
    cpt->pressure = orig_pt->pressure;
    cpt->strength = orig_pt->strength;
    copy_v4_v4(cpt->vert_color, orig_pt->vert_color);

    /* default handle type */
    bezt->h1 |= HD_ALIGN;
    bezt->h2 |= HD_ALIGN;

    cpt->point_index = r_cubic_orig_index[i];
  }

  MEM_freeN(points);
  if (r_cubic_array) {
    free(r_cubic_array);
  }
  if (r_corners_index_array) {
    free(r_corners_index_array);
  }
  if (r_cubic_orig_index) {
    free(r_cubic_orig_index);
  }

  return editcurve;
}

/**
 * Updates the editcurve for a stroke.
 */
void BKE_gpencil_stroke_editcurve_update(bGPDstroke *gps, float error_threshold)
{
  if (gps == NULL || gps->totpoints < 0) {
    return;
  }

  if (gps->editcurve != NULL) {
    BKE_gpencil_free_stroke_editcurve(gps);
  }

  bGPDcurve *editcurve = BKE_gpencil_stroke_editcurve_generate(gps, error_threshold);
  if (editcurve == NULL) {
    return;
  }
  /* update the selection based on the selected points in the stroke */
  BKE_gpencil_editcurve_stroke_sync_selection(gps, editcurve);
  gps->editcurve = editcurve;
}

/**
 * Sync the selection from stroke to editcurve
 */
void BKE_gpencil_editcurve_stroke_sync_selection(bGPDstroke *gps, bGPDcurve *gpc)
{
  if (gps->flag & GP_STROKE_SELECT) {
    gpc->flag |= GP_CURVE_SELECT;
  }
  else {
    gpc->flag &= ~GP_CURVE_SELECT;
  }

  for (int i = 0; i < gpc->tot_curve_points; i++) {
    bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
    bGPDspoint *pt = &gps->points[gpc_pt->point_index];
    if (pt->flag & GP_SPOINT_SELECT) {
      gpc_pt->flag |= GP_CURVE_POINT_SELECT;
      BEZT_SEL_ALL(&gpc_pt->bezt);
    }
    else {
      gpc_pt->flag &= ~GP_CURVE_POINT_SELECT;
      BEZT_DESEL_ALL(&gpc_pt->bezt);
    }
  }
}

/**
 * Sync the selection from editcurve to stroke
 */
void BKE_gpencil_stroke_editcurve_sync_selection(bGPDstroke *gps, bGPDcurve *gpc)
{
  if (gpc->flag & GP_CURVE_SELECT) {
    gps->flag |= GP_STROKE_SELECT;
  }
  else {
    gps->flag &= ~GP_STROKE_SELECT;
  }

  for (int i = 0; i < gpc->tot_curve_points; i++) {
    bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
    bGPDspoint *pt = &gps->points[gpc_pt->point_index];

    if (gpc_pt->flag & GP_CURVE_POINT_SELECT) {
      pt->flag |= GP_SPOINT_SELECT;
      if (i + 1 < gpc->tot_curve_points) {
        bGPDcurve_point *gpc_pt_next = &gpc->curve_points[i + 1];
        if (gpc_pt_next->flag & GP_CURVE_POINT_SELECT) {
          /* select all the points inbetween */
          for (int j = gpc_pt->point_index + 1; j < gpc_pt_next->point_index; j++) {
            bGPDspoint *pt_next = &gps->points[j];
            pt_next->flag |= GP_SPOINT_SELECT;
          }
        }
      }
    }
  }
}

/**
 * Update editcurve for all selected strokes.
 */
void BKE_gpencil_selected_strokes_editcurve_update(bGPdata *gpd)
{
  if (gpd == NULL) {
    return;
  }

  const bool is_multiedit = (bool)GPENCIL_MULTIEDIT_SESSIONS_ON(gpd);

  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    if (!BKE_gpencil_layer_is_editable(gpl)) {
      continue;
    }
    bGPDframe *init_gpf = (is_multiedit) ? gpl->frames.first : gpl->actframe;
    for (bGPDframe *gpf = init_gpf; gpf; gpf = gpf->next) {
      if ((gpf == gpl->actframe) || ((gpf->flag & GP_FRAME_SELECT) && is_multiedit)) {
        LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
          /* skip deselected stroke */
          if (!(gps->flag & GP_STROKE_SELECT)) {
            continue;
          }

          if (gps->editcurve == NULL) {
            BKE_gpencil_stroke_editcurve_update(gps, gpd->curve_edit_threshold);
            if (gps->editcurve != NULL) {
              gps->editcurve->resolution = gpd->editcurve_resolution;
              gps->editcurve->flag |= GP_CURVE_RECALC_GEOMETRY;
            }
          }
          BKE_gpencil_stroke_geometry_update(gpd, gps);
        }
      }
    }
  }
}

static void gpencil_interpolate_fl_from_to(
    float from, float to, float *point_offset, int it, int stride)
{
  /* linear interpolation */
  float *r = point_offset;
  for (int i = 0; i <= it; i++) {
    float fac = (float)i / (float)it;
    *r = interpf(to, from, fac);
    r = POINTER_OFFSET(r, stride);
  }
}

static void gpencil_interpolate_v4_from_to(
    float from[4], float to[4], float *point_offset, int it, int stride)
{
  /* linear interpolation */
  float *r = point_offset;
  for (int i = 0; i <= it; i++) {
    float fac = (float)i / (float)it;
    interp_v4_v4v4(r, from, to, fac);
    r = POINTER_OFFSET(r, stride);
  }
}

/**
 * Recalculate stroke points with the editcurve of the stroke.
 */
void BKE_gpencil_stroke_update_geometry_from_editcurve(bGPDstroke *gps)
{
  if (gps == NULL || gps->editcurve == NULL) {
    return;
  }

  bGPDcurve *editcurve = gps->editcurve;
  bGPDcurve_point *curve_point_array = editcurve->curve_points;
  int curve_point_array_len = editcurve->tot_curve_points;
  int resolu = editcurve->resolution;
  bool is_cyclic = gps->flag & GP_STROKE_CYCLIC;

  const uint array_last = curve_point_array_len - 1;
  /* One stride contains: x, y, z, pressure, strength, Vr, Vg, Vb, Vmix_factor */
  const uint stride = sizeof(float[9]);
  const uint resolu_stride = resolu * stride;
  const uint points_len = BKE_curve_calc_coords_axis_len(
      curve_point_array_len, resolu, is_cyclic, true);

  float(*points)[9] = MEM_callocN((stride * points_len * (is_cyclic ? 2 : 1)), __func__);
  float *points_offset = &points[0][0];
  for (unsigned int i = 0; i < array_last; i++) {
    bGPDcurve_point *cpt_curr = &curve_point_array[i];
    bGPDcurve_point *cpt_next = &curve_point_array[i + 1];

    /* sample points on all 3 axis between two curve points */
    for (uint axis = 0; axis < 3; axis++) {
      BKE_curve_forward_diff_bezier(cpt_curr->bezt.vec[1][axis],
                                    cpt_curr->bezt.vec[2][axis],
                                    cpt_next->bezt.vec[0][axis],
                                    cpt_next->bezt.vec[1][axis],
                                    POINTER_OFFSET(points_offset, sizeof(float) * axis),
                                    (int)resolu,
                                    stride);
    }

    /* interpolate other attributes */
    gpencil_interpolate_fl_from_to(cpt_curr->pressure,
                                   cpt_next->pressure,
                                   POINTER_OFFSET(points_offset, sizeof(float) * 3),
                                   resolu,
                                   stride);
    gpencil_interpolate_fl_from_to(cpt_curr->strength,
                                   cpt_next->strength,
                                   POINTER_OFFSET(points_offset, sizeof(float) * 4),
                                   resolu,
                                   stride);
    gpencil_interpolate_v4_from_to(cpt_curr->vert_color,
                                   cpt_next->vert_color,
                                   POINTER_OFFSET(points_offset, sizeof(float) * 5),
                                   resolu,
                                   stride);
    /* update the index */
    cpt_curr->point_index = i * resolu;
    points_offset = POINTER_OFFSET(points_offset, resolu_stride);
  }

  /* TODO: make cyclic strokes work */
  // if (is_cyclic) {
  //   bGPDcurve_point *cpt_curr = &curve_point_array[array_last];
  //   bGPDcurve_point *cpt_next = &curve_point_array[0];
  //   BKE_curve_forward_diff_bezier(cpt_curr->bezt.vec[1][axis],
  //                                 cpt_curr->bezt.vec[2][axis],
  //                                 cpt_next->bezt.vec[0][axis],
  //                                 cpt_next->bezt.vec[1][axis],
  //                                 points_offset,
  //                                 (int)resolu,
  //                                 stride);
  //   points_offset = POINTER_OFFSET(points_offset, stride);
  // }
  // else {
  //   float *points_last = POINTER_OFFSET(&points[0][axis], array_last * resolu_stride);
  //   *points_last = curve_point_array[array_last].bezt.vec[1][axis];
  //   points_offset = POINTER_OFFSET(points_offset, stride);
  // }

  if (is_cyclic) {
    memcpy(points[points_len], points[0], stride * points_len);
  }

  /* resize stroke point array */
  gps->totpoints = points_len;
  gps->points = MEM_recallocN(gps->points, sizeof(bGPDspoint) * gps->totpoints);
  if (gps->dvert != NULL) {
    gps->dvert = MEM_recallocN(gps->dvert, sizeof(MDeformVert) * gps->totpoints);
  }

  /* write new data to stroke point array */
  for (int i = 0; i < points_len; i++) {
    bGPDspoint *pt = &gps->points[i];
    copy_v3_v3(&pt->x, &points[i][0]);

    pt->pressure = points[i][3];
    pt->strength = points[i][4];

    copy_v4_v4(pt->vert_color, &points[i][5]);
  }

  MEM_freeN(points);
}

/**
 * Recalculate the handles of the edit curve of a grease pencil stroke
 */
void BKE_gpencil_editcurve_recalculate_handles(bGPDstroke *gps)
{
  if (gps == NULL || gps->editcurve == NULL) {
    return;
  }

  bool changed = false;
  bGPDcurve *gpc = gps->editcurve;
  for (int i = 0; i < gpc->tot_curve_points; i++) {
    bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
    if (gpc_pt->flag & GP_CURVE_POINT_SELECT) {
      bGPDcurve_point *gpc_pt_prev = (i > 0) ? &gpc->curve_points[i - 1] : NULL;
      bGPDcurve_point *gpc_pt_next = (i < gpc->tot_curve_points - 1) ? &gpc->curve_points[i + 1] :
                                                                       NULL;

      BezTriple *bezt = &gpc_pt->bezt;
      BezTriple *bezt_prev = gpc_pt_prev != NULL ? &gpc_pt_prev->bezt : NULL;
      BezTriple *bezt_next = gpc_pt_next != NULL ? &gpc_pt_next->bezt : NULL;

      BKE_nurb_handle_calc(bezt, bezt_prev, bezt_next, false, 0);
      changed = true;
    }
  }

  if (changed) {
    gpc->flag |= GP_CURVE_RECALC_GEOMETRY;
  }
}

/** \} */
