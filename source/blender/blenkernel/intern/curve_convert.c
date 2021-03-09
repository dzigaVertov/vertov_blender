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
 */

/** \file
 * \ingroup bke
 */
#include <string.h>
#include "MEM_guardedalloc.h"
#include "BLI_string.h"
#include "DNA_curve_types.h"
#include "DNA_object_types.h"
#include "DNA_vfont_types.h"

#include "BLI_utildefines.h"
#include "BLI_blenlib.h"

#include "BKE_curve.h"
#include "BKE_curveprofile.h"
#include "BKE_displist.h"
#include "BKE_font.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_main.h"
#include "BKE_modifier.h"


#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"



static Curve *curve_from_font_object(Object *object, Depsgraph *depsgraph)
{
  Curve *curve = (Curve *)object->data;
  Curve *new_curve = (Curve *)BKE_id_copy_ex(NULL, &curve->id, NULL, LIB_ID_COPY_LOCALIZE);

  Object *evaluated_object = DEG_get_evaluated_object(depsgraph, object);
  BKE_vfont_to_curve_nubase(evaluated_object, FO_EDIT, &new_curve->nurb);

  new_curve->type = OB_CURVE;

  new_curve->flag &= ~CU_3D;
  BKE_curve_curve_dimension_update(new_curve);

  return new_curve;
}

static Curve *curve_from_curve_object(Object *object, Depsgraph *depsgraph, bool apply_modifiers)
{
  Object *evaluated_object = DEG_get_evaluated_object(depsgraph, object);
  Curve *curve = (Curve *)evaluated_object->data;
  Curve *new_curve = (Curve *)BKE_id_copy_ex(NULL, &curve->id, NULL, LIB_ID_COPY_LOCALIZE);

  if (apply_modifiers) {
    BKE_curve_calc_modifiers_pre(depsgraph,
                                 DEG_get_input_scene(depsgraph),
                                 evaluated_object,
                                 BKE_curve_nurbs_get(curve),
                                 &new_curve->nurb,
                                 DEG_get_mode(depsgraph) == DAG_EVAL_RENDER);
  }

  return new_curve;
}


Curve *BKE_curve_new_from_object(Object *object, Depsgraph *depsgraph, bool apply_modifiers)
{
  if (!ELEM(object->type, OB_FONT, OB_CURVE)) {
    return NULL;
  }

  if (object->type == OB_FONT) {
    return curve_from_font_object(object, depsgraph);
    
  }

  return curve_from_curve_object(object, depsgraph, apply_modifiers);
}

/* Poser adding new curve from object */
static int foreach_libblock_make_original_callback(LibraryIDLinkCallbackData *cb_data)
{
  ID **id_p = cb_data->id_pointer;
  if (*id_p == NULL) {
    return IDWALK_RET_NOP;
  }
  *id_p = DEG_get_original_id(*id_p);

  return IDWALK_RET_NOP;
}

static int foreach_libblock_make_usercounts_callback(LibraryIDLinkCallbackData *cb_data)
{
  ID **id_p = cb_data->id_pointer;
  if (*id_p == NULL) {
    return IDWALK_RET_NOP;
  }

  const int cb_flag = cb_data->cb_flag;
  if (cb_flag & IDWALK_CB_USER) {
    id_us_plus(*id_p);
  }
  else if (cb_flag & IDWALK_CB_USER_ONE) {
    /* Note: in that context, that one should not be needed (since there should be at least already
     * one USER_ONE user of that ID), but better be consistent. */
    id_us_ensure_real(*id_p);
  }
  return IDWALK_RET_NOP;
}

void BKE_curve_nomain_to_curve(Curve *curve_src,
			       Curve *curve_dst,
			       Object *ob,
			       bool take_ownership)
{
  BLI_assert(curve_src->id.tag & LIB_TAG_NO_MAIN);

  /* Poser blindly following BKE_mesh_nomain_to_mesh */
  Curve *tmp = curve_dst;
  BLI_listbase_clear(&tmp->nurb);
  BKE_nurbList_duplicate(&(tmp->nurb), &(curve_src->nurb));
  
  tmp->mat = MEM_dupallocN(curve_src->mat);

  tmp->str = MEM_dupallocN(curve_src->str);
  tmp->strinfo = MEM_dupallocN(curve_src->strinfo);
  tmp->tb = MEM_dupallocN(curve_src->tb);
  tmp->batch_cache = NULL;

  tmp->bevel_profile = BKE_curveprofile_copy(curve_src->bevel_profile);

  tmp->editnurb = NULL;
  tmp->editfont = NULL;

  /* skip the listbase */
  MEMCPY_STRUCT_AFTER(curve_dst, tmp, id.prev);
  BKE_id_free(NULL, curve_src);
}


Curve *BKE_curve_new_from_object_to_bmain(Main *bmain,
                                        Depsgraph *depsgraph,
                                        Object *object,
                                        bool preserve_all_data_layers)
{
  BLI_assert(ELEM(object->type, OB_CURVE));

  Curve *curve = BKE_curve_new_from_object(object, depsgraph, preserve_all_data_layers);
  if (curve == NULL) {
    /* Unable to convert the object to a curve, return an empty one. */
    Curve *curve_in_bmain = BKE_curve_add(bmain, ((ID *)object->data)->name + 2, 1);
    id_us_min(&curve_in_bmain->id);
    return curve_in_bmain;
  }

  /* Make sure mesh only points original datablocks, also increase users of materials and other
   * possibly referenced data-blocks.
   *
   * Going to original data-blocks is required to have bmain in a consistent state, where
   * everything is only allowed to reference original data-blocks.
   *
   * Note that user-count updates has to be done *after* mesh has been transferred to Main database
   * (since doing refcounting on non-Main IDs is forbidden). */
  BKE_library_foreach_ID_link(
      NULL, &curve->id, foreach_libblock_make_original_callback, NULL, IDWALK_NOP);

  /* Append the mesh to 'bmain'.
   * We do it a bit longer way since there is no simple and clear way of adding existing data-block
   * to the 'bmain'. So we allocate new empty mesh in the 'bmain' (which guarantees all the naming
   * and orders and flags) and move the temporary mesh in place there. */
  Curve *curve_in_bmain = BKE_curve_add(bmain, curve->id.name + 2, OB_CURVE);

  /* NOTE: BKE_mesh_nomain_to_mesh() does not copy materials and instead it preserves them in the
   * destination mesh. So we "steal" all related fields before calling it.
   *
   * TODO(sergey): We really better have a function which gets and ID and accepts it for the bmain.
   */
  /* poser see if this is necessary */
  /* mesh_in_bmain->mat = mesh->mat; */
  curve_in_bmain->mat = curve->mat;
  /* mesh_in_bmain->totcol = mesh->totcol; */
  curve_in_bmain->totcol = curve->totcol;
    
  /* mesh_in_bmain->flag = mesh->flag; */
  curve_in_bmain->flag = curve->flag;
  /* mesh_in_bmain->smoothresh = mesh->smoothresh; */
  /* mesh->mat = NULL; */

  BKE_curve_nomain_to_curve(curve, curve_in_bmain, NULL, true);

  /* User-count is required because so far mesh was in a limbo, where library management does
   * not perform any user management (i.e. copy of a mesh will not increase users of materials). */
  BKE_library_foreach_ID_link(
      NULL, &curve_in_bmain->id, foreach_libblock_make_usercounts_callback, NULL, IDWALK_NOP);

  /* Make sure user count from BKE_mesh_add() is the one we expect here and bring it down to 0. */
  BLI_assert(curve_in_bmain->id.us == 1);
  id_us_min(&curve_in_bmain->id);

  return curve_in_bmain;
}
