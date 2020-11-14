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
 * Copyright 2019, Blender Foundation.
 */

/** \file
 * \ingroup draw_engine
 */
#include "DRW_render.h"

#include "draw_cache_impl.h"
#include "draw_manager_text.h"

#include "BKE_editmesh.h"
#include "BKE_image.h"

#include "DNA_mesh_types.h"

#include "ED_image.h"

#include "GPU_batch.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "overlay_private.h"

typedef struct OVERLAY_StretchingAreaTotals {
  void *next, *prev;
  float *total_area;
  float *total_area_uv;
} OVERLAY_StretchingAreaTotals;

static OVERLAY_UVLineStyle edit_uv_line_style_from_space_image(const SpaceImage *sima)
{
  const bool is_uv_editor = sima->mode == SI_MODE_UV;
  if (is_uv_editor) {
    switch (sima->dt_uv) {
      case SI_UVDT_OUTLINE:
        return OVERLAY_UV_LINE_STYLE_OUTLINE;
      case SI_UVDT_BLACK:
        return OVERLAY_UV_LINE_STYLE_BLACK;
      case SI_UVDT_WHITE:
        return OVERLAY_UV_LINE_STYLE_WHITE;
      case SI_UVDT_DASH:
        return OVERLAY_UV_LINE_STYLE_DASH;
      default:
        return OVERLAY_UV_LINE_STYLE_BLACK;
    }
  }
  else {
    return OVERLAY_UV_LINE_STYLE_SHADOW;
  }
}

/* -------------------------------------------------------------------- */
/** \name Internal API
 * \{ */

void OVERLAY_edit_uv_init(OVERLAY_Data *vedata)
{
  OVERLAY_StorageList *stl = vedata->stl;
  OVERLAY_PrivateData *pd = stl->pd;
  const DRWContextState *draw_ctx = DRW_context_state_get();
  SpaceImage *sima = (SpaceImage *)draw_ctx->space_data;
  const Scene *scene = draw_ctx->scene;
  const ToolSettings *ts = scene->toolsettings;
  const bool show_overlays = !pd->hide_overlays;

  Image *image = sima->image;
  /* By design no image is an image type. This so editor shows UV's by default. */
  const bool is_image_type =
      (image == NULL) || ELEM(image->type, IMA_TYPE_IMAGE, IMA_TYPE_MULTILAYER, IMA_TYPE_UV_TEST);
  const bool is_uv_editor = sima->mode == SI_MODE_UV;
  const bool has_edit_object = (draw_ctx->object_edit) != NULL;
  const bool is_paint_mode = sima->mode == SI_MODE_PAINT;
  const bool is_view_mode = sima->mode == SI_MODE_VIEW;
  const bool is_edit_mode = draw_ctx->object_mode == OB_MODE_EDIT;
  const bool do_uv_overlay = is_image_type && is_uv_editor && has_edit_object;
  const bool show_modified_uvs = sima->flag & SI_DRAWSHADOW;
  const bool is_tiled_image = image && (image->source == IMA_SRC_TILED);
  const bool do_faces = ((sima->flag & SI_NO_DRAWFACES) == 0);
  const bool do_face_dots = (ts->uv_flag & UV_SYNC_SELECTION) ?
                                (ts->selectmode & SCE_SELECT_FACE) != 0 :
                                (ts->uv_selectmode == UV_SELECT_FACE);
  const bool do_uvstretching_overlay = is_image_type && is_uv_editor && is_edit_mode &&
                                       ((sima->flag & SI_DRAW_STRETCH) != 0);
  const bool do_tex_paint_shadows = (sima->flag & SI_NO_DRAW_TEXPAINT) == 0;

  pd->edit_uv.do_faces = show_overlays && do_faces && !do_uvstretching_overlay;
  pd->edit_uv.do_face_dots = show_overlays && do_faces && do_face_dots;
  pd->edit_uv.do_uv_overlay = show_overlays && do_uv_overlay;
  pd->edit_uv.do_uv_shadow_overlay = show_overlays && is_image_type &&
                                     ((is_paint_mode && do_tex_paint_shadows &&
                                       ((draw_ctx->object_mode &
                                         (OB_MODE_TEXTURE_PAINT | OB_MODE_EDIT)) != 0)) ||
                                      (is_view_mode && do_tex_paint_shadows &&
                                       ((draw_ctx->object_mode & (OB_MODE_TEXTURE_PAINT)) != 0)) ||
                                      (do_uv_overlay && (show_modified_uvs)));
  pd->edit_uv.do_uv_stretching_overlay = show_overlays && do_uvstretching_overlay;
  pd->edit_uv.uv_opacity = sima->uv_opacity;
  pd->edit_uv.do_tiled_image_overlay = show_overlays && is_image_type && is_tiled_image;
  pd->edit_uv.do_tiled_image_border_overlay = is_image_type && is_tiled_image;
  pd->edit_uv.dash_length = 4.0f * UI_DPI_FAC;
  pd->edit_uv.line_style = edit_uv_line_style_from_space_image(sima);
  pd->edit_uv.do_smooth_wire = ((U.gpu_flag & USER_GPU_FLAG_OVERLAY_SMOOTH_WIRE) > 0);

  pd->edit_uv.draw_type = sima->dt_uvstretch;
  BLI_listbase_clear(&pd->edit_uv.totals);
  pd->edit_uv.total_area_ratio = 0.0f;
  pd->edit_uv.total_area_ratio_inv = 0.0f;

  ED_space_image_get_uv_aspect(sima, &pd->edit_uv.aspect[0], &pd->edit_uv.aspect[1]);
}

void OVERLAY_edit_uv_cache_init(OVERLAY_Data *vedata)
{
  OVERLAY_StorageList *stl = vedata->stl;
  OVERLAY_PassList *psl = vedata->psl;
  OVERLAY_PrivateData *pd = stl->pd;

  if (pd->edit_uv.do_uv_overlay || pd->edit_uv.do_uv_shadow_overlay) {
    /* uv edges */
    {
      DRW_PASS_CREATE(psl->edit_uv_edges_ps,
                      DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                          DRW_STATE_BLEND_ALPHA);
      GPUShader *sh = OVERLAY_shader_edit_uv_edges_get();
      if (pd->edit_uv.do_uv_shadow_overlay) {
        pd->edit_uv_shadow_edges_grp = DRW_shgroup_create(sh, psl->edit_uv_edges_ps);
        DRW_shgroup_uniform_block(pd->edit_uv_shadow_edges_grp, "globalsBlock", G_draw.block_ubo);
        DRW_shgroup_uniform_int_copy(
            pd->edit_uv_shadow_edges_grp, "lineStyle", OVERLAY_UV_LINE_STYLE_SHADOW);
        DRW_shgroup_uniform_float_copy(
            pd->edit_uv_shadow_edges_grp, "alpha", pd->edit_uv.uv_opacity);
        DRW_shgroup_uniform_float(
            pd->edit_uv_shadow_edges_grp, "dashLength", &pd->edit_uv.dash_length, 1);
        DRW_shgroup_uniform_bool(
            pd->edit_uv_shadow_edges_grp, "doSmoothWire", &pd->edit_uv.do_smooth_wire, 1);
      }

      if (pd->edit_uv.do_uv_overlay) {
        pd->edit_uv_edges_grp = DRW_shgroup_create(sh, psl->edit_uv_edges_ps);
        DRW_shgroup_uniform_block(pd->edit_uv_edges_grp, "globalsBlock", G_draw.block_ubo);
        DRW_shgroup_uniform_int_copy(pd->edit_uv_edges_grp, "lineStyle", pd->edit_uv.line_style);
        DRW_shgroup_uniform_float_copy(pd->edit_uv_edges_grp, "alpha", pd->edit_uv.uv_opacity);
        DRW_shgroup_uniform_float(
            pd->edit_uv_edges_grp, "dashLength", &pd->edit_uv.dash_length, 1);
        DRW_shgroup_uniform_bool(
            pd->edit_uv_edges_grp, "doSmoothWire", &pd->edit_uv.do_smooth_wire, 1);
      }
    }
  }

  if (pd->edit_uv.do_uv_overlay) {
    /* uv verts */
    {
      DRW_PASS_CREATE(psl->edit_uv_verts_ps,
                      DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                          DRW_STATE_BLEND_ALPHA);
      GPUShader *sh = OVERLAY_shader_edit_uv_verts_get();
      pd->edit_uv_verts_grp = DRW_shgroup_create(sh, psl->edit_uv_verts_ps);

      const float point_size = UI_GetThemeValuef(TH_VERTEX_SIZE);

      DRW_shgroup_uniform_block(pd->edit_uv_verts_grp, "globalsBlock", G_draw.block_ubo);
      DRW_shgroup_uniform_float_copy(
          pd->edit_uv_verts_grp, "pointSize", (point_size + 1.5f) * M_SQRT2);
      DRW_shgroup_uniform_float_copy(pd->edit_uv_verts_grp, "outlineWidth", 0.75f);
    }

    /* uv faces */
    if (pd->edit_uv.do_faces) {
      DRW_PASS_CREATE(psl->edit_uv_faces_ps,
                      DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS | DRW_STATE_BLEND_ALPHA);
      GPUShader *sh = OVERLAY_shader_edit_uv_face_get();
      pd->edit_uv_faces_grp = DRW_shgroup_create(sh, psl->edit_uv_faces_ps);
      DRW_shgroup_uniform_block(pd->edit_uv_faces_grp, "globalsBlock", G_draw.block_ubo);
      DRW_shgroup_uniform_float(pd->edit_uv_faces_grp, "uvOpacity", &pd->edit_uv.uv_opacity, 1);
    }

    /* uv face dots */
    if (pd->edit_uv.do_face_dots) {
      const float point_size = UI_GetThemeValuef(TH_FACEDOT_SIZE);
      GPUShader *sh = OVERLAY_shader_edit_uv_face_dots_get();
      pd->edit_uv_face_dots_grp = DRW_shgroup_create(sh, psl->edit_uv_verts_ps);
      DRW_shgroup_uniform_block(pd->edit_uv_face_dots_grp, "globalsBlock", G_draw.block_ubo);
      DRW_shgroup_uniform_float_copy(pd->edit_uv_face_dots_grp, "pointSize", point_size);
    }
  }

  /* uv stretching */
  if (pd->edit_uv.do_uv_stretching_overlay) {
    DRW_PASS_CREATE(psl->edit_uv_stretching_ps,
                    DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS | DRW_STATE_BLEND_ALPHA);
    if (pd->edit_uv.draw_type == SI_UVDT_STRETCH_ANGLE) {
      GPUShader *sh = OVERLAY_shader_edit_uv_stretching_angle_get();
      pd->edit_uv_stretching_grp = DRW_shgroup_create(sh, psl->edit_uv_stretching_ps);
      DRW_shgroup_uniform_block(pd->edit_uv_stretching_grp, "globalsBlock", G_draw.block_ubo);
      DRW_shgroup_uniform_vec2_copy(pd->edit_uv_stretching_grp, "aspect", pd->edit_uv.aspect);
    }
    else /* SI_UVDT_STRETCH_AREA */ {
      GPUShader *sh = OVERLAY_shader_edit_uv_stretching_area_get();
      pd->edit_uv_stretching_grp = DRW_shgroup_create(sh, psl->edit_uv_stretching_ps);
      DRW_shgroup_uniform_block(pd->edit_uv_stretching_grp, "globalsBlock", G_draw.block_ubo);
      DRW_shgroup_uniform_float(
          pd->edit_uv_stretching_grp, "totalAreaRatio", &pd->edit_uv.total_area_ratio, 1);
      DRW_shgroup_uniform_float(
          pd->edit_uv_stretching_grp, "totalAreaRatioInv", &pd->edit_uv.total_area_ratio_inv, 1);
    }
  }

  if (pd->edit_uv.do_tiled_image_border_overlay) {
    const DRWContextState *draw_ctx = DRW_context_state_get();
    SpaceImage *sima = (SpaceImage *)draw_ctx->space_data;
    Image *image = sima->image;
    GPUBatch *geom = DRW_cache_quad_wires_get();
    float obmat[4][4];
    unit_m4(obmat);

    DRW_PASS_CREATE(psl->edit_uv_tiled_image_borders_ps,
                    DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_ALWAYS);
    GPUShader *sh = OVERLAY_shader_edit_uv_tiled_image_borders_get();

    float theme_color[4], selected_color[4];
    UI_GetThemeColorShade4fv(TH_BACK, 60, theme_color);
    UI_GetThemeColor4fv(TH_FACE_SELECT, selected_color);
    srgb_to_linearrgb_v4(theme_color, theme_color);
    srgb_to_linearrgb_v4(selected_color, selected_color);

    DRWShadingGroup *grp = DRW_shgroup_create(sh, psl->edit_uv_tiled_image_borders_ps);
    DRW_shgroup_uniform_vec4_copy(grp, "color", theme_color);
    DRW_shgroup_uniform_vec3_copy(grp, "offset", (float[3]){0.0f, 0.0f, 0.0f});

    LISTBASE_FOREACH (ImageTile *, tile, &image->tiles) {
      const int tile_x = ((tile->tile_number - 1001) % 10);
      const int tile_y = ((tile->tile_number - 1001) / 10);
      obmat[3][1] = (float)tile_y;
      obmat[3][0] = (float)tile_x;
      DRW_shgroup_call_obmat(grp, geom, obmat);
    }
    /* Only mark active border when overlays are enabled. */
    if (pd->edit_uv.do_tiled_image_overlay) {
      /* Active tile border */
      ImageTile *active_tile = BLI_findlink(&image->tiles, image->active_tile_index);
      obmat[3][0] = (float)((active_tile->tile_number - 1001) % 10);
      obmat[3][1] = (float)((active_tile->tile_number - 1001) / 10);
      grp = DRW_shgroup_create(sh, psl->edit_uv_tiled_image_borders_ps);
      DRW_shgroup_uniform_vec4_copy(grp, "color", selected_color);
      DRW_shgroup_call_obmat(grp, geom, obmat);
    }
  }

  if (pd->edit_uv.do_tiled_image_overlay) {
    const DRWContextState *draw_ctx = DRW_context_state_get();
    SpaceImage *sima = (SpaceImage *)draw_ctx->space_data;
    Image *image = sima->image;

    struct DRWTextStore *dt = DRW_text_cache_ensure();
    uchar color[4];
    /* Color Management: Exception here as texts are drawn in sRGB space directly.  */
    UI_GetThemeColorShade4ubv(TH_BACK, 60, color);
    char text[16];
    LISTBASE_FOREACH (ImageTile *, tile, &image->tiles) {
      BLI_snprintf(text, 5, "%d", tile->tile_number);
      float tile_location[3] = {
          ((tile->tile_number - 1001) % 10), ((tile->tile_number - 1001) / 10), 0.0f};
      DRW_text_cache_add(dt,
                         tile_location,
                         text,
                         strlen(text),
                         10,
                         10,
                         DRW_TEXT_CACHE_GLOBALSPACE | DRW_TEXT_CACHE_ASCII,
                         color);
    }
  }
}

void OVERLAY_edit_uv_cache_populate(OVERLAY_Data *vedata, Object *ob)
{
  OVERLAY_StorageList *stl = vedata->stl;
  OVERLAY_PrivateData *pd = stl->pd;
  GPUBatch *geom;

  const DRWContextState *draw_ctx = DRW_context_state_get();
  const bool is_edit_object = DRW_object_is_in_edit_mode(ob);
  Mesh *me = (Mesh *)ob->data;
  const bool has_active_object_uvmap = CustomData_get_active_layer(&me->ldata, CD_MLOOPUV) != -1;
  const bool has_active_edit_uvmap = is_edit_object &&
                                     (CustomData_get_active_layer(&me->edit_mesh->bm->ldata,
                                                                  CD_MLOOPUV) != -1);
  const bool draw_shadows = (draw_ctx->object_mode != OB_MODE_OBJECT) &&
                            (ob->mode == draw_ctx->object_mode);

  if (has_active_edit_uvmap) {
    if (pd->edit_uv.do_uv_overlay) {
      geom = DRW_mesh_batch_cache_get_edituv_edges(ob->data);
      if (geom) {
        DRW_shgroup_call_obmat(pd->edit_uv_edges_grp, geom, NULL);
      }
      geom = DRW_mesh_batch_cache_get_edituv_verts(ob->data);
      if (geom) {
        DRW_shgroup_call_obmat(pd->edit_uv_verts_grp, geom, NULL);
      }
      if (pd->edit_uv.do_faces) {
        geom = DRW_mesh_batch_cache_get_edituv_faces(ob->data);
        if (geom) {
          DRW_shgroup_call_obmat(pd->edit_uv_faces_grp, geom, NULL);
        }
      }
      if (pd->edit_uv.do_face_dots) {
        geom = DRW_mesh_batch_cache_get_edituv_facedots(ob->data);
        if (geom) {
          DRW_shgroup_call_obmat(pd->edit_uv_face_dots_grp, geom, NULL);
        }
      }
    }

    if (pd->edit_uv.do_uv_stretching_overlay) {
      if (pd->edit_uv.draw_type == SI_UVDT_STRETCH_ANGLE) {
        geom = DRW_mesh_batch_cache_get_edituv_faces_stretch_angle(me);
      }
      else /* SI_UVDT_STRETCH_AREA */ {
        OVERLAY_StretchingAreaTotals *totals = MEM_mallocN(sizeof(OVERLAY_StretchingAreaTotals),
                                                           __func__);
        BLI_addtail(&pd->edit_uv.totals, totals);
        geom = DRW_mesh_batch_cache_get_edituv_faces_stretch_area(
            me, &totals->total_area, &totals->total_area_uv);
      }
      if (geom) {
        DRW_shgroup_call_obmat(pd->edit_uv_stretching_grp, geom, NULL);
      }
    }
  }

  if (draw_shadows && (has_active_object_uvmap || has_active_edit_uvmap)) {
    if (pd->edit_uv.do_uv_shadow_overlay) {
      geom = DRW_mesh_batch_cache_get_uv_edges(ob->data);
      if (geom) {
        DRW_shgroup_call_obmat(pd->edit_uv_shadow_edges_grp, geom, NULL);
      }
    }
  }
}

static void edit_uv_stretching_update_ratios(OVERLAY_Data *vedata)
{
  OVERLAY_StorageList *stl = vedata->stl;
  OVERLAY_PrivateData *pd = stl->pd;

  if (pd->edit_uv.draw_type == SI_UVDT_STRETCH_AREA) {
    float total_area = 0.0f;
    float total_area_uv = 0.0f;

    LISTBASE_FOREACH (OVERLAY_StretchingAreaTotals *, totals, &pd->edit_uv.totals) {
      total_area += *totals->total_area;
      total_area_uv += *totals->total_area_uv;
    }

    if (total_area > FLT_EPSILON && total_area_uv > FLT_EPSILON) {
      pd->edit_uv.total_area_ratio = total_area / total_area_uv;
      pd->edit_uv.total_area_ratio_inv = total_area_uv / total_area;
    }
  }
  BLI_freelistN(&pd->edit_uv.totals);
}

void OVERLAY_edit_uv_draw(OVERLAY_Data *vedata)
{
  OVERLAY_PassList *psl = vedata->psl;
  OVERLAY_StorageList *stl = vedata->stl;
  OVERLAY_PrivateData *pd = stl->pd;

  if (pd->edit_uv.do_tiled_image_border_overlay) {
    DRW_draw_pass(psl->edit_uv_tiled_image_borders_ps);
  }

  if (pd->edit_uv.do_uv_stretching_overlay) {
    edit_uv_stretching_update_ratios(vedata);
    DRW_draw_pass(psl->edit_uv_stretching_ps);
  }
  if (pd->edit_uv.do_uv_overlay) {
    if (pd->edit_uv.do_faces) {
      DRW_draw_pass(psl->edit_uv_faces_ps);
    }
    DRW_draw_pass(psl->edit_uv_edges_ps);

    DRW_draw_pass(psl->edit_uv_verts_ps);
  }
  else if (pd->edit_uv.do_uv_shadow_overlay) {
    DRW_draw_pass(psl->edit_uv_edges_ps);
  }
}

/* \{ */
