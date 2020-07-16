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
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 */

/** \file
 * \ingroup edtransform
 */

#include "DNA_gpencil_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_math.h"

#include "BKE_colortools.h"
#include "BKE_context.h"
#include "BKE_curve.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_curve.h"
#include "BKE_gpencil_geom.h"

#include "ED_gpencil.h"

#include "transform.h"
#include "transform_convert.h"

/* -------------------------------------------------------------------- */
/** \name Gpencil Transform Creation
 *
 * \{ */

static void createTransGPencil_curve_center_get(bGPDcurve *gpc, float r_center[3])
{
  zero_v3(r_center);
  int tot_sel = 0;
  for (int i = 0; i < gpc->tot_curve_points; i++) {
    bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
    if (gpc_pt->flag & GP_CURVE_POINT_SELECT) {
      BezTriple *bezt = &gpc_pt->bezt;
      /* only allow rotation around control point for now... */
      if (bezt->f2 & SELECT) {
        add_v3_v3(r_center, bezt->vec[1]);
        tot_sel++;
      }
    }
  }

  if (tot_sel > 0) {
    mul_v3_fl(r_center, 1.0f / tot_sel);
  }
}

static void createTransGPencil_center_get(bGPDstroke *gps, float r_center[3])
{
  bGPDspoint *pt;
  int i;

  zero_v3(r_center);
  int tot_sel = 0;
  for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
    if (pt->flag & GP_SPOINT_SELECT) {
      add_v3_v3(r_center, &pt->x);
      tot_sel++;
    }
  }

  if (tot_sel > 0) {
    mul_v3_fl(r_center, 1.0f / tot_sel);
  }
}

void createTransGPencil(bContext *C, TransInfo *t)
{
  if (t->data_container_len == 0) {
    return;
  }

  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  const Scene *scene = CTX_data_scene(C);
  ToolSettings *ts = scene->toolsettings;
  Object *obact = CTX_data_active_object(C);
  bGPdata *gpd = obact->data;
  BLI_assert(gpd != NULL);

  TransData *td = NULL;
  float mtx[3][3], smtx[3][3];

  const int cfra_scene = CFRA;

  const bool is_multiedit = (bool)GPENCIL_MULTIEDIT_SESSIONS_ON(gpd);
  const bool use_multiframe_falloff = (ts->gp_sculpt.flag & GP_SCULPT_SETT_FLAG_FRAME_FALLOFF) !=
                                      0;
  const bool is_curve_edit = (bool)GPENCIL_CURVE_EDIT_SESSIONS_ON(gpd);

  const bool is_prop_edit = (t->flag & T_PROP_EDIT) != 0;
  const bool is_prop_edit_connected = (t->flag & T_PROP_CONNECTED) != 0;
  const bool is_scale_thickness = ((t->mode == TFM_GPENCIL_SHRINKFATTEN) ||
                                   (ts->gp_sculpt.flag & GP_SCULPT_SETT_FLAG_SCALE_THICKNESS));

  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);

  /* .Grease Pencil Strokes to Transform Data.  */
  tc->data_len = 0;

  /* initialize falloff curve */
  if (is_multiedit) {
    BKE_curvemapping_initialize(ts->gp_sculpt.cur_falloff);
  }

  /* First Pass: Count the number of data-points required for the strokes,
   * (and additional info about the configuration - e.g. 2D/3D?).
   */
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* Only editable and visible layers are considered. */
    if (BKE_gpencil_layer_is_editable(gpl) && (gpl->actframe != NULL)) {
      bGPDframe *gpf;
      bGPDframe *init_gpf = (is_multiedit) ? gpl->frames.first : gpl->actframe;

      for (gpf = init_gpf; gpf; gpf = gpf->next) {
        if ((gpf == gpl->actframe) || ((gpf->flag & GP_FRAME_SELECT) && (is_multiedit))) {
          LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
            /* skip strokes that are invalid for current view */
            if (ED_gpencil_stroke_can_use(C, gps) == false) {
              continue;
            }
            /* Check if the color is editable. */
            if (ED_gpencil_stroke_color_use(obact, gpl, gps) == false) {
              continue;
            }

            if (is_curve_edit && gps->editcurve != NULL) {
              bGPDcurve *gpc = gps->editcurve;
              if (is_prop_edit) {
                if (is_prop_edit_connected) {
                  if (gpc->flag & GP_CURVE_SELECT) {
                    tc->data_len += gpc->tot_curve_points * 3;
                  }
                }
                else {
                  tc->data_len += gpc->tot_curve_points * 3;
                }
              }
              else {
                for (int i = 0; i < gpc->tot_curve_points; i++) {
                  bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
                  if (gpc_pt->flag & GP_CURVE_POINT_SELECT) {
                    BezTriple *bezt = &gpc_pt->bezt;
                    /* if control point is selected, treat handles as selected */
                    if (bezt->f2 & SELECT) {
                      tc->data_len += 3;
                      continue;
                    }
                    if (bezt->f1 & SELECT) {
                      tc->data_len++;
                    }
                    if (bezt->f3 & SELECT) {
                      tc->data_len++;
                    }
                  }
                }
              }
            }
            else {
              if (is_prop_edit) {
                /* Proportional Editing... */
                if (is_prop_edit_connected) {
                  /* Connected only - so only if selected. */
                  if (gps->flag & GP_STROKE_SELECT) {
                    tc->data_len += gps->totpoints;
                  }
                }
                else {
                  /* Everything goes - connection status doesn't matter. */
                  tc->data_len += gps->totpoints;
                }
              }
              else {
                /* Only selected stroke points are considered. */
                if (gps->flag & GP_STROKE_SELECT) {
                  bGPDspoint *pt;
                  int i;

                  for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
                    if (pt->flag & GP_SPOINT_SELECT) {
                      tc->data_len++;
                    }
                  }
                }
              }
            }
          }
        }
        /* If not multiedit out of loop. */
        if (!is_multiedit) {
          break;
        }
      }
    }
  }

  /* Stop trying if nothing selected. */
  if (tc->data_len == 0) {
    return;
  }

  /* Allocate memory for data */
  tc->data = MEM_callocN(tc->data_len * sizeof(TransData), "TransData(GPencil)");
  td = tc->data;

  unit_m3(smtx);
  unit_m3(mtx);

  /* Second Pass: Build transdata array. */
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* only editable and visible layers are considered */
    if (BKE_gpencil_layer_is_editable(gpl) && (gpl->actframe != NULL)) {
      const int cfra = (gpl->flag & GP_LAYER_FRAMELOCK) ? gpl->actframe->framenum : cfra_scene;
      bGPDframe *gpf = gpl->actframe;
      float diff_mat[4][4];
      float inverse_diff_mat[4][4];

      bGPDframe *init_gpf = (is_multiedit) ? gpl->frames.first : gpl->actframe;
      /* Init multiframe falloff options. */
      int f_init = 0;
      int f_end = 0;

      if (use_multiframe_falloff) {
        BKE_gpencil_frame_range_selected(gpl, &f_init, &f_end);
      }

      /* Calculate difference matrix. */
      BKE_gpencil_parent_matrix_get(depsgraph, obact, gpl, diff_mat);
      /* Undo matrix. */
      invert_m4_m4(inverse_diff_mat, diff_mat);

      /* Make a new frame to work on if the layer's frame
       * and the current scene frame don't match up.
       *
       * - This is useful when animating as it saves that "uh-oh" moment when you realize you've
       *   spent too much time editing the wrong frame...
       */
      if ((gpf->framenum != cfra) && (!is_multiedit)) {
        gpf = BKE_gpencil_frame_addcopy(gpl, cfra);
        /* in some weird situations (framelock enabled) return NULL */
        if (gpf == NULL) {
          continue;
        }
        if (!is_multiedit) {
          init_gpf = gpf;
        }
      }

      /* Loop over strokes, adding TransData for points as needed... */
      for (gpf = init_gpf; gpf; gpf = gpf->next) {
        if ((gpf == gpl->actframe) || ((gpf->flag & GP_FRAME_SELECT) && (is_multiedit))) {

          /* If multi-frame and falloff, recalculate and save value. */
          float falloff = 1.0f; /* by default no falloff */
          if ((is_multiedit) && (use_multiframe_falloff)) {
            /* Falloff depends on distance to active frame
             * (relative to the overall frame range). */
            falloff = BKE_gpencil_multiframe_falloff_calc(
                gpf, gpl->actframe->framenum, f_init, f_end, ts->gp_sculpt.cur_falloff);
          }

          LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
            TransData *head = td;
            TransData *tail = td;
            bGPDcurve *gpc = gps->editcurve;
            bool stroke_ok;
            int tot_points;

            /* skip strokes that are invalid for current view */
            if (ED_gpencil_stroke_can_use(C, gps) == false) {
              continue;
            }
            /* check if the color is editable */
            if (ED_gpencil_stroke_color_use(obact, gpl, gps) == false) {
              continue;
            }

            if (is_curve_edit && gpc != NULL) {
              if (is_prop_edit) {
                if (is_prop_edit_connected) {
                  stroke_ok = (gpc->flag & GP_CURVE_SELECT) != 0;
                }
                else {
                  stroke_ok = true;
                }
              }
              else {
                stroke_ok = (gpc->flag & GP_CURVE_SELECT) != 0;
              }
              tot_points = gpc->tot_curve_points;
            }
            else {
              /* What we need to include depends on proportional editing settings... */
              if (is_prop_edit) {
                if (is_prop_edit_connected) {
                  /* A) "Connected" - Only those in selected strokes */
                  stroke_ok = (gps->flag & GP_STROKE_SELECT) != 0;
                }
                else {
                  /* B) All points, always */
                  stroke_ok = true;
                }
              }
              else {
                /* C) Only selected points in selected strokes */
                stroke_ok = (gps->flag & GP_STROKE_SELECT) != 0;
              }
              tot_points = gps->totpoints;
            }

            /* Do stroke... */
            if (stroke_ok && tot_points > 0) {
              float center[3];
              bool point_ok;

              /* save falloff factor */
              gps->runtime.multi_frame_falloff = falloff;

              if (is_curve_edit) {
                createTransGPencil_curve_center_get(gpc, center);

                for (int i = 0; i < tot_points; i++) {
                  bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
                  if (is_prop_edit) {
                    point_ok = true;
                  }
                  else {
                    point_ok = (gpc_pt->flag & GP_CURVE_POINT_SELECT) != 0;
                  }

                  if (point_ok) {
                    BezTriple *bezt = &gpc_pt->bezt;
                    for (int j = 0; j < 3; j++) {
                      td->flag = 0;
                      /* always do transform if control point is selected or if proportional
                       * editing is enabled. Otherwise only look at selected handles */
                      if (bezt->f2 & SELECT || BEZT_ISSEL_IDX(bezt, j) || is_prop_edit) {
                        copy_v3_v3(td->iloc, bezt->vec[j]);
                        if ((gpc->flag & GP_CURVE_SELECT) &&
                            (ts->transform_pivot_point == V3D_AROUND_LOCAL_ORIGINS)) {
                          copy_v3_v3(td->center, center);
                        }
                        else {
                          copy_v3_v3(td->center, bezt->vec[j]);
                        }

                        td->loc = bezt->vec[j];
                        td->flag |= TD_SELECTED;

                        /* can only change thickness and strength if control point is selected */
                        if (j == 1) {
                          if (t->mode != TFM_MIRROR) {
                            if (t->mode != TFM_GPENCIL_OPACITY) {
                              if (is_scale_thickness) {
                                td->val = &gpc_pt->pressure;
                                td->ival = gpc_pt->pressure;
                              }
                            }
                            else {
                              td->val = &gpc_pt->strength;
                              td->ival = gpc_pt->strength;
                            }
                          }
                        }

                        /* apply parent transformations */
                        copy_m3_m4(td->smtx, inverse_diff_mat); /* final position */
                        copy_m3_m4(td->mtx, diff_mat);          /* display position */
                        copy_m3_m4(td->axismtx, diff_mat);      /* axis orientation */

                        /* Save the stroke for recalc geometry function. */
                        td->extra = gps;

                        /* Save pointer to object. */
                        td->ob = obact;

                        td++;
                        tail++;
                      }
                    }
                  }
                }
              }
              else {
                /* calculate stroke center */
                createTransGPencil_center_get(gps, center);

                for (int i = 0; i < tot_points; i++) {
                  bGPDspoint *pt = &gps->points[i];

                  /* include point? */
                  if (is_prop_edit) {
                    /* Always all points in strokes that get included. */
                    point_ok = true;
                  }
                  else {
                    /* Only selected points in selected strokes. */
                    point_ok = (pt->flag & GP_SPOINT_SELECT) != 0;
                  }

                  /* do point... */
                  if (point_ok) {
                    copy_v3_v3(td->iloc, &pt->x);
                    /* Only copy center in local origins.
                     * This allows get interesting effects also when move
                     * using proportional editing. */
                    if ((gps->flag & GP_STROKE_SELECT) &&
                        (ts->transform_pivot_point == V3D_AROUND_LOCAL_ORIGINS)) {
                      copy_v3_v3(td->center, center);
                    }
                    else {
                      copy_v3_v3(td->center, &pt->x);
                    }

                    td->loc = &pt->x;

                    td->flag = 0;

                    if (pt->flag & GP_SPOINT_SELECT) {
                      td->flag |= TD_SELECTED;
                    }

                    /* For other transform modes (e.g. shrink-fatten), need to additional data
                     * but never for mirror.
                     */
                    if (t->mode != TFM_MIRROR) {
                      if (t->mode != TFM_GPENCIL_OPACITY) {
                        if (is_scale_thickness) {
                          td->val = &pt->pressure;
                          td->ival = pt->pressure;
                        }
                      }
                      else {
                        td->val = &pt->strength;
                        td->ival = pt->strength;
                      }
                    }
#if 0
                    /* screenspace needs special matrices... */
                    if ((gps->flag & (GP_STROKE_3DSPACE | GP_STROKE_2DSPACE | GP_STROKE_2DIMAGE)) ==
                        0) {
                      /* screenspace */
                      td->protectflag = OB_LOCK_LOCZ | OB_LOCK_ROTZ | OB_LOCK_SCALEZ;
                    }
                    else {
                      /* configure 2D dataspace points so that they don't play up... */
                      if (gps->flag & (GP_STROKE_2DSPACE | GP_STROKE_2DIMAGE)) {
                        td->protectflag = OB_LOCK_LOCZ | OB_LOCK_ROTZ | OB_LOCK_SCALEZ;
                      }
                    }
#endif
                    /* apply parent transformations */
                    copy_m3_m4(td->smtx, inverse_diff_mat); /* final position */
                    copy_m3_m4(td->mtx, diff_mat);          /* display position */
                    copy_m3_m4(td->axismtx, diff_mat);      /* axis orientation */

                    /* Save the stroke for recalc geometry function. */
                    td->extra = gps;

                    /* Save pointer to object. */
                    td->ob = obact;

                    td++;
                    tail++;
                  }
                }
              }
              /* March over these points, and calculate the proportional editing distances. */
              if (is_prop_edit && (head != tail)) {
                calc_distanceCurveVerts(head, tail - 1, false);
              }
            }
          }
        }
        /* If not multiedit out of loop. */
        if (!is_multiedit) {
          break;
        }
      }
    }
  }
}

/* force recalculation of triangles during transformation */
void recalcData_gpencil_strokes(TransInfo *t)
{
  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);
  GHash *strokes = BLI_ghash_ptr_new(__func__);

  TransData *td = tc->data;
  bGPdata *gpd = td->ob->data;
  for (int i = 0; i < tc->data_len; i++, td++) {
    bGPDstroke *gps = td->extra;

    if ((gps != NULL) && (!BLI_ghash_haskey(strokes, gps))) {
      if (GPENCIL_CURVE_EDIT_SESSIONS_ON(gpd) && gps->editcurve != NULL) {
        BKE_gpencil_editcurve_recalculate_handles(gps);
        gps->flag |= GP_STROKE_NEEDS_CURVE_UPDATE;
      }
      /* Calc geometry data. */
      BKE_gpencil_stroke_geometry_update(gpd, gps);
    }
  }
  BLI_ghash_free(strokes, NULL, NULL);
}

/** \} */
