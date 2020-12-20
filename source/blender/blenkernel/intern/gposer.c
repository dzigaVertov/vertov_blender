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
 * \ingroup bke
 */

#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_alloca.h"
#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"
#include "BLT_translation.h"

#include "DNA_armature_types.h"
#include "DNA_constraint_types.h"
#include "DNA_listBase.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_action.h"
#include "BKE_anim_visualization.h"
#include "BKE_armature.h"
#include "BKE_constraint.h"
#include "BKE_curve.h"
#include "BKE_context.h"
#include "BKE_idprop.h"
#include "BKE_idtype.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_main.h"
#include "BKE_object.h"
#include "BKE_scene.h"

#include "DEG_depsgraph_build.h"
#include "DEG_depsgraph_query.h"

#include "BIK_api.h"

#include "CLG_log.h"

/*****************************************************************************/
/*                                Draw Engine                                */
/*****************************************************************************/
void (*BKE_gposer_batch_cache_dirty_tag_cb)(bArmature *arm) = NULL;
void (*BKE_gposer_batch_cache_free_cb)(bArmature *arm) = NULL;

void BKE_gposer_batch_cache_dirty_tag(bArmature *arm){
  if (arm) {
    DEG_id_tag_update(&arm->id, ID_RECALC_GEOMETRY);
    BKE_gposer_batch_cache_dirty_tag_cb(arm);
  }
  
}


/* ****************** Gposer Handle calculation ************** */
/* Duplicated code from curve.c  */
/* TODO: Figure out what to do with this */
void BKE_gposer_calchandleNurb_intern(Bone *bone,
				      const eBezTriple_Flag__Alias handle_sel_flag,
				      const bool is_fcurve,
				      const bool skip_align,
				      const char fcurve_smoothing)
{
  BezTriple *bezt = &bone->bezt;
  /* defines to avoid confusion */
  #define p2_h1 ((p2)-3)
  #define p2_h2 ((p2) + 3)
    
  BezTriple *prev = bone->bbone_prev ? &bone->bbone_prev->bezt : NULL;
  BezTriple *next = bone->bbone_next ? &bone->bbone_next->bezt : NULL;


  const float *p1, *p3;
  float *p2;
  float pt[3];
  float dvec_a[3], dvec_b[3];
  float len, len_a, len_b;
  float len_ratio;
  const float eps = 1e-5;

  /* assume normal handle until we check */
  bezt->auto_handle_type = HD_AUTOTYPE_NORMAL;

  if (bezt->h1 == 0 && bezt->h2 == 0) {
    return;
  }

  p2 = bezt->vec[1];

  if (prev == NULL) {
    p3 = next->vec[1];
    pt[0] = 2.0f * p2[0] - p3[0];
    pt[1] = 2.0f * p2[1] - p3[1];
    pt[2] = 2.0f * p2[2] - p3[2];
    p1 = pt;
  }
  else {
    p1 = prev->vec[1];
  }

  if (next == NULL) {
    pt[0] = 2.0f * p2[0] - p1[0];
    pt[1] = 2.0f * p2[1] - p1[1];
    pt[2] = 2.0f * p2[2] - p1[2];
    p3 = pt;
  }
  else {
    p3 = next->vec[1];
  }

  sub_v3_v3v3(dvec_a, p2, p1);
  sub_v3_v3v3(dvec_b, p3, p2);

  if (is_fcurve) {
    len_a = dvec_a[0];
    len_b = dvec_b[0];
  }
  else {
    len_a = len_v3(dvec_a);
    len_b = len_v3(dvec_b);
  }

  if (len_a == 0.0f) {
    len_a = 1.0f;
  }
  if (len_b == 0.0f) {
    len_b = 1.0f;
  }

  len_ratio = len_a / len_b;

  if (ELEM(bezt->h1, HD_AUTO, HD_AUTO_ANIM) || ELEM(bezt->h2, HD_AUTO, HD_AUTO_ANIM)) { /* auto */
    float tvec[3];
    tvec[0] = dvec_b[0] / len_b + dvec_a[0] / len_a;
    tvec[1] = dvec_b[1] / len_b + dvec_a[1] / len_a;
    tvec[2] = dvec_b[2] / len_b + dvec_a[2] / len_a;

    len = len_v3(tvec);
    len *= 2.5614f;

    if (len != 0.0f) {
      /* only for fcurves */
      bool leftviolate = false, rightviolate = false;

      if (ELEM(bezt->h1, HD_AUTO, HD_AUTO_ANIM)) {
        len_a /= len;
        madd_v3_v3v3fl(p2_h1, p2, tvec, -len_a);

        if ((bezt->h1 == HD_AUTO_ANIM) && next && prev) { /* keep horizontal if extrema */
          float ydiff1 = prev->vec[1][1] - bezt->vec[1][1];
          float ydiff2 = next->vec[1][1] - bezt->vec[1][1];
          if ((ydiff1 <= 0.0f && ydiff2 <= 0.0f) || (ydiff1 >= 0.0f && ydiff2 >= 0.0f)) {
            bezt->vec[0][1] = bezt->vec[1][1];
            bezt->auto_handle_type = HD_AUTOTYPE_LOCKED_FINAL;
          }
          else { /* handles should not be beyond y coord of two others */
            if (ydiff1 <= 0.0f) {
              if (prev->vec[1][1] > bezt->vec[0][1]) {
                bezt->vec[0][1] = prev->vec[1][1];
                leftviolate = 1;
              }
            }
            else {
              if (prev->vec[1][1] < bezt->vec[0][1]) {
                bezt->vec[0][1] = prev->vec[1][1];
                leftviolate = 1;
              }
            }
          }
        }
      }
      if (ELEM(bezt->h2, HD_AUTO, HD_AUTO_ANIM)) {
        len_b /= len;
        madd_v3_v3v3fl(p2_h2, p2, tvec, len_b);

        if ((bezt->h2 == HD_AUTO_ANIM) && next && prev) { /* keep horizontal if extrema */
          float ydiff1 = prev->vec[1][1] - bezt->vec[1][1];
          float ydiff2 = next->vec[1][1] - bezt->vec[1][1];
          if ((ydiff1 <= 0.0f && ydiff2 <= 0.0f) || (ydiff1 >= 0.0f && ydiff2 >= 0.0f)) {
            bezt->vec[2][1] = bezt->vec[1][1];
            bezt->auto_handle_type = HD_AUTOTYPE_LOCKED_FINAL;
          }
          else { /* handles should not be beyond y coord of two others */
            if (ydiff1 <= 0.0f) {
              if (next->vec[1][1] < bezt->vec[2][1]) {
                bezt->vec[2][1] = next->vec[1][1];
                rightviolate = 1;
              }
            }
            else {
              if (next->vec[1][1] > bezt->vec[2][1]) {
                bezt->vec[2][1] = next->vec[1][1];
                rightviolate = 1;
              }
            }
          }
        }
      }
      if (leftviolate || rightviolate) { /* align left handle */
        BLI_assert(is_fcurve);
        /* simple 2d calculation */
        float h1_x = p2_h1[0] - p2[0];
        float h2_x = p2[0] - p2_h2[0];

        if (leftviolate) {
          p2_h2[1] = p2[1] + ((p2[1] - p2_h1[1]) / h1_x) * h2_x;
        }
        else {
          p2_h1[1] = p2[1] + ((p2[1] - p2_h2[1]) / h2_x) * h1_x;
        }
      }
    }
  }

  if (bezt->h1 == HD_VECT) { /* vector */
    madd_v3_v3v3fl(p2_h1, p2, dvec_a, -1.0f / 3.0f);
  }
  if (bezt->h2 == HD_VECT) {
    madd_v3_v3v3fl(p2_h2, p2, dvec_b, 1.0f / 3.0f);
  }

  if (skip_align ||
      /* when one handle is free, alignming makes no sense, see: T35952 */
      (ELEM(HD_FREE, bezt->h1, bezt->h2)) ||
      /* also when no handles are aligned, skip this step */
      (!ELEM(HD_ALIGN, bezt->h1, bezt->h2) && !ELEM(HD_ALIGN_DOUBLESIDE, bezt->h1, bezt->h2))) {
    /* handles need to be updated during animation and applying stuff like hooks,
     * but in such situations it's quite difficult to distinguish in which order
     * align handles should be aligned so skip them for now */
    return;
  }

  len_a = len_v3v3(p2, p2_h1);
  len_b = len_v3v3(p2, p2_h2);

  if (len_a == 0.0f) {
    len_a = 1.0f;
  }
  if (len_b == 0.0f) {
    len_b = 1.0f;
  }

  len_ratio = len_a / len_b;

  if (bezt->f1 & handle_sel_flag) {                      /* order of calculation */
    if (ELEM(bezt->h2, HD_ALIGN, HD_ALIGN_DOUBLESIDE)) { /* aligned */
      if (len_a > eps) {
        len = 1.0f / len_ratio;
        p2_h2[0] = p2[0] + len * (p2[0] - p2_h1[0]);
        p2_h2[1] = p2[1] + len * (p2[1] - p2_h1[1]);
        p2_h2[2] = p2[2] + len * (p2[2] - p2_h1[2]);
      }
    }
    if (ELEM(bezt->h1, HD_ALIGN, HD_ALIGN_DOUBLESIDE)) {
      if (len_b > eps) {
        len = len_ratio;
        p2_h1[0] = p2[0] + len * (p2[0] - p2_h2[0]);
        p2_h1[1] = p2[1] + len * (p2[1] - p2_h2[1]);
        p2_h1[2] = p2[2] + len * (p2[2] - p2_h2[2]);
      }
    }
  }
  else {
    if (ELEM(bezt->h1, HD_ALIGN, HD_ALIGN_DOUBLESIDE)) {
      if (len_b > eps) {
        len = len_ratio;
        p2_h1[0] = p2[0] + len * (p2[0] - p2_h2[0]);
        p2_h1[1] = p2[1] + len * (p2[1] - p2_h2[1]);
        p2_h1[2] = p2[2] + len * (p2[2] - p2_h2[2]);
      }
    }
    if (ELEM(bezt->h2, HD_ALIGN, HD_ALIGN_DOUBLESIDE)) { /* aligned */
      if (len_a > eps) {
        len = 1.0f / len_ratio;
        p2_h2[0] = p2[0] + len * (p2[0] - p2_h1[0]);
        p2_h2[1] = p2[1] + len * (p2[1] - p2_h1[1]);
        p2_h2[2] = p2[2] + len * (p2[2] - p2_h1[2]);
      }
    }
  }

#undef p2_h1
#undef p2_h2
  
}


/**
 * @brief      Update gposer control's beztriple with the position of the handle bones
 *
 * @param      Bone *bone
 * @param      bPoseChannel *pchan
 * @param      bPose *poseparam
 *
 * @return     void
 */
void BKE_gposer_update_bone_beztriple(struct Bone *bone,
				      struct bPoseChannel *pchan,
				      struct bPose *pose)
{
  if (bone->poser_flag & IS_CONTROL){
    BezTriple *bezt = &(bone->bezt);
    float *pos_control = bezt->vec[1];
    float pchan_mtx_in[4][4];
    float pchan_mtx_out[4][4];
    BKE_pchan_to_mat4(pchan, pchan_mtx_in);
    BKE_armature_mat_bone_to_pose(pchan, pchan_mtx_in, pchan_mtx_out);
    copy_v3_v3(pos_control, pchan_mtx_out[3]);
  }
  else if (bone->poser_flag & IS_HANDLE_LEFT){
    float *pos_lhandle = bone->gp_lhandle->bezt.vec[0];
    float pchan_mtx_in[4][4];
    float pchan_mtx_out[4][4];
    BKE_pchan_to_mat4(pchan, pchan_mtx_in);
    BKE_armature_mat_bone_to_pose(pchan, pchan_mtx_in, pchan_mtx_out);
    copy_v3_v3(pos_lhandle, pchan_mtx_out[3]);
  } else if (bone->poser_flag & IS_HANDLE_RIGHT){
    float *pos_rhandle = bone->gp_lhandle->bezt.vec[2];
    float pchan_mtx_in[4][4];
    float pchan_mtx_out[4][4];
    BKE_pchan_to_mat4(pchan, pchan_mtx_in);
    BKE_armature_mat_bone_to_pose(pchan, pchan_mtx_in, pchan_mtx_out);
    copy_v3_v3(pos_rhandle, pchan_mtx_out[3]);
  }    
}


/**
 * @brief      Updates all gposer controls beztriples based on handle bones positions
 *
 * @param      bContext *C
 *
 * @return     void
 */
void BKE_gposer_update_bones_beztriples(struct bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  bPose *pose = ob->pose;
  
  CTX_DATA_BEGIN(C, bPoseChannel *, pchan, visible_pose_bones){
    Bone *bone = pchan->bone;

    if ((bone->poser_flag & (IS_CONTROL | IS_HANDLE_LEFT | IS_HANDLE_RIGHT)) != 0){
      BKE_gposer_update_bone_beztriple(bone, pchan, pose);      
    }
  }
  CTX_DATA_END;  
}


/**
 * @brief      Copies position from control bone beztriple to handle bones
 *
 * @param      Bone *bone
 * @param      bPoseChannel *pchan
 * @param      bPose *pose
 *
 * @return     void
 */
void BKE_gposer_update_handle(struct Bone *bone,
			      struct bPoseChannel *pchan,
			      struct bPose *pose)
{
  if (bone->gp_lhandle){
    char *lhandle_name = bone->gp_lhandle->name;
    
    bPoseChannel *lhandle_pchan = BKE_pose_channel_find_name(pose, lhandle_name);
    float *pos_lhandle = bone->bezt.vec[0];
    
    float pchan_mtx_in[4][4];
    float pchan_mtx_out[4][4];
    BKE_pchan_to_mat4(lhandle_pchan, pchan_mtx_in);
    BKE_armature_mat_bone_to_pose(lhandle_pchan, pchan_mtx_in, pchan_mtx_out);
    copy_v3_v3(pchan_mtx_out[3], pos_lhandle);
    BKE_armature_mat_pose_to_bone(lhandle_pchan, pchan_mtx_out, pchan_mtx_in);
    BKE_pchan_apply_mat4(lhandle_pchan, pchan_mtx_in, false);
  }
  
  if (bone->gp_rhandle){
    char *rhandle_name = bone->gp_rhandle->name;
    
    bPoseChannel *rhandle_pchan = BKE_pose_channel_find_name(pose, rhandle_name);
    float *pos_rhandle = bone->bezt.vec[2];
    float pchan_mtx_in[4][4];
    float pchan_mtx_out[4][4];
    BKE_pchan_to_mat4(rhandle_pchan, pchan_mtx_in);
    BKE_armature_mat_bone_to_pose(rhandle_pchan, pchan_mtx_in, pchan_mtx_out);
    copy_v3_v3(pchan_mtx_out[3], pos_rhandle);
    BKE_armature_mat_pose_to_bone(rhandle_pchan, pchan_mtx_out, pchan_mtx_in);
    BKE_pchan_apply_mat4(rhandle_pchan, pchan_mtx_in, false);
  }	    
}  


/**
 * @brief      Copies positions of the handles from the control's bones beztriples to the handle bones
 *
 *
 * @param      bContext *C
 *
 * @return     void
 */
void BKE_gposer_update_handles(struct bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  bPose *pose = ob->pose;

  CTX_DATA_BEGIN(C, bPoseChannel *, pchan, visible_pose_bones){
    Bone *bone = pchan->bone;
    if ((bone->poser_flag & IS_CONTROL) != 0){
      BKE_gposer_update_handle(bone, pchan, pose);      
    }
  }
  CTX_DATA_END;
}
