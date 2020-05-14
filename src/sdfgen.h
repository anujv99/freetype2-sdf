
#ifndef SDFGEN_H_
#define SDFGEN_H_

#include <ft2build.h>
#include FT_OUTLINE_H
#include FT_BITMAP_H

FT_BEGIN_HEADER

  // generate sdf from outline
  FT_EXPORT(FT_Error)
  FT_Generate_SDF( FT_Library   library,
                   FT_Outline*  outline, 
                   FT_Bitmap   *abitmap );


  /* ------------------- private stuff ------------------- */

  // float vector
  typedef struct  SDF_Vector_
  {
    float  x;
    float  y;

  } SDF_Vector;

  /* enumeration for the types of contour present in FT_Outline */
  typedef enum  SDF_Contour_Type_
  {
    SDF_CONTOUR_TYPE_LINE,              /* line type contour */
    SDF_CONTOUR_TYPE_QUADRATIC_BEZIER,  /* quadratic bezier type contour */
    SDF_CONTOUR_TYPE_CUBIC_BEZIER,      /* cubic bezier type contour */

    SDF_CONTOUR_TYPE_MAX

  } SDF_Contour_Type;

  /* structure to hold a contour */
  typedef struct  SDF_Contour_
  {
    SDF_Vector  start_pos;         /* all contour type has a start position */
    SDF_Vector  end_pos;           /* all contour type has a end position */

    SDF_Vector  control_point_a;   /* unused in line type contour */
    SDF_Vector  control_point_b;   /* unused in line and quadratic bezier type contour */

    SDF_Contour_Type contour_type  /* contour identifier */

  } SDF_Contour;

FT_END_HEADER

#endif /* SDFGEN_H_ */

/* END */
