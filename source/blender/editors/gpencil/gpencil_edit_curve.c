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
 * Operators for editing Grease Pencil strokes
 */

/** \file
 * \ingroup edgpencil
 */

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "MEM_guardedalloc.h"

#include "DNA_gpencil_types.h"
#include "DNA_view3d_types.h"

#include "BKE_context.h"
#include "BKE_gpencil.h"

#include "BLI_listbase.h"
#include "BLI_math.h"

#include "WM_api.h"
#include "WM_types.h"

#include "ED_gpencil.h"

#include "DEG_depsgraph.h"

#include "gpencil_intern.h"

/* -------------------------------------------------------------------- */
/** \name Test Operator for curve editing
 * \{ */

static bGPDcurve *create_example_gp_curve(void)
{
  int num_points = 2;
  bGPDcurve *new_gp_curve = (bGPDcurve *)MEM_callocN(sizeof(bGPDcurve), __func__);
  new_gp_curve->tot_curve_points = num_points;
  new_gp_curve->curve_points = (BezTriple *)MEM_callocN(sizeof(BezTriple) * num_points, __func__);
  new_gp_curve->point_index_array = (int *)MEM_callocN(sizeof(int) * num_points, __func__);

  /* We just write some recognizable data to the BezTriple */
  for (int i = 0; i < num_points; ++i) {
    BezTriple *bezt = &new_gp_curve->curve_points[i];
    for (int j = 0; j < 3; ++j) {
      copy_v3_fl3(bezt->vec[j], i, j, i * j);
    }
    bezt->radius = 1.0f;
    bezt->weight = 2.0f;
  }
  return new_gp_curve;
}

static int gp_write_stroke_curve_data_exec(bContext *C, wmOperator *op)
{
  Object *ob = CTX_data_active_object(C);
  bGPdata *gpd = ob->data;

  if (ELEM(NULL, gpd)) {
    return OPERATOR_CANCELLED;
  }

  bGPDlayer *gpl = BKE_gpencil_layer_active_get(gpd);
  bGPDframe *gpf = gpl->actframe;
  if (ELEM(NULL, gpf)) {
    return OPERATOR_CANCELLED;
  }

  LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
    gps->editcurve = create_example_gp_curve();
  }

  /* notifiers */
  DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, NULL);

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_write_sample_stroke_curve_data(wmOperatorType *ot)
{
  PropertyRNA *prop;

  /* identifiers */
  ot->name = "Write sample stroke curve data";
  ot->idname = "GPENCIL_OT_write_stroke_curve_data";
  ot->description = "Test operator to write to the curve data in a grease pencil stroke.";

  /* api callbacks */
  ot->exec = gp_write_stroke_curve_data_exec;
  ot->poll = gp_active_layer_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  /* XXX: no props for now */
}

/** \} */