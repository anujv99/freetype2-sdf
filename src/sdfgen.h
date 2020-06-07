
#ifndef SDFGEN_H_
#define SDFGEN_H_

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H
#include FT_BITMAP_H

FT_BEGIN_HEADER

  typedef FT_Vector FT_26D6Vec;
  typedef FT_Vector FT_16D16Vec;

  /* generate sdf from outline */
  /* input: library, outline   */
  /* output: abitmap           */
  FT_EXPORT( FT_Error )
  Generate_SDF( FT_Library     library,
                FT_GlyphSlot   glyph,
                FT_Bitmap     *abitmap );


  /* Private Stuff */

  /**************************************************************************
   *
   * SDF Outline implementation structs and enums.
   *
   */

  /* can be used to determine weather the point is */
  /* inside or outside the shape's outline         */
  typedef struct SDF_Signed_Distance_
  {
    FT_Int       sign;           /* weather outside or inside    */
    FT_Fixed     distance;       /* magnitude of `nearest_point' */

  } SDF_Signed_Distance;

  /* enumeration for the types of edge present in FT_Outline */
  typedef enum  SDF_Edge_Type_
  {
    SDF_EDGE_TYPE_NONE              = -1,
    SDF_EDGE_TYPE_LINE              =  0,  /* line type             */
    SDF_EDGE_TYPE_QUADRATIC_BEZIER  =  1,  /* quadratic bezier type */
    SDF_EDGE_TYPE_CUBIC_BEZIER      =  2,  /* cubic bezier type     */

  } SDF_Edge_Type;

  typedef enum  SDF_Contour_Orientation_
  {
    SDF_CONTOUR_ORIENTATION_NONE            = 0,
    SDF_CONTOUR_ORIENTATION_CLOCKWISE       = 1,
    SDF_CONTOUR_ORIENTATION_ANTI_CLOCKWISE  = 2,

  } SDF_Contour_Orientation;

  /* structure to hold a edge */
  typedef struct  SDF_Edge_
  {
    FT_26D6Vec            start_pos;        /* start position of edge      */
    FT_26D6Vec            end_pos;          /* end position of edge        */
    FT_26D6Vec            control_point_a;  /* unused in line type         */
    FT_26D6Vec            control_point_b;  /* unused in line & quadratic  */

    SDF_Edge_Type         edge_type;        /* edge identifier             */

    struct SDF_Edge_*     next;             /* to create a linked list     */

  } SDF_Edge;

  /* structure to hold a contour which is made of several edges */
  typedef struct  SDF_Contour_
  {
    FT_26D6Vec               last_pos;    /* endpoint of last edge   */

    SDF_Contour_Orientation  orientation; /* orientation of contour  */

    SDF_Edge*                head;        /* linked list of edges    */

    FT_ULong                 num_edges;   /* total number of edges   */

    struct SDF_Contour_*     next;        /* to create a linked list */

  } SDF_Contour;

  /* structure represent a complete shape defined by FT_Outline */
  /* in a form of linked list of contours                       */
  typedef struct SDF_Shape_
  {         
    SDF_Contour*  head;          /* linked list of all the edges  */

    FT_ULong      num_contours;  /* total number of contours      */

    FT_Memory     memory;        /* to allocate/deallocate memory */

  } SDF_Shape;


  /**************************************************************************
   *
   * SDF Outline implementation functions.
   *
   */

  FT_LOCAL( void )
  SDF_Edge_Init( SDF_Edge  *edge );

  FT_LOCAL( void )
  SDF_Contour_Init( SDF_Contour  *contour );

  FT_LOCAL( void )
  SDF_Shape_Init( SDF_Shape  *shape );

  /* no need to create SDF_Edge_Done or SDF_Contour_Done becuase */
  /* SDF_Shape_Done will free all the allocated SDF_Contour and  */
  /* SDF_Edge                                                    */
  FT_LOCAL( FT_Error )
  SDF_Shape_Done( SDF_Shape  *shape );

  FT_LOCAL( FT_Error )
  SDF_Decompose_Outline( FT_Outline*  outline,
                         SDF_Shape   *shape );

  /**************************************************************************
   *
   * Math functions
   *
   */

  /* returns the orientation of the contour the orintation */
  /* is determined by calculating the area of the control  */
  /* box. positive area is taken clockwise.                */
  FT_LOCAL( SDF_Contour_Orientation )
  get_contour_orientation( SDF_Contour*  contour );

  /* returns the signed distance of a point on the `contour' */
  /* that is nearest to `point'                              */
  FT_LOCAL( FT_Error )
  get_min_conour( SDF_Contour*          contour,
                  const FT_26D6Vec      point,
                  SDF_Signed_Distance  *out );

  /* returns the signed distance of a point on the curve `edge' */
  /* that is nearest to `point'                                 */
  FT_LOCAL( FT_Error )
  get_min_distance( SDF_Edge*             edge,
                    const FT_26D6Vec      point,
                    SDF_Signed_Distance  *out );

FT_END_HEADER

#endif /* SDFGEN_H_ */

/* END */
