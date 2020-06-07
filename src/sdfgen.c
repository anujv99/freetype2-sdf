
#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include <stdlib.h>

#include <ft2build.h>

#include FT_FREETYPE_H
#include FT_ERRORS_H
#include FT_INTERNAL_OBJECTS_H
#include FT_INTERNAL_CALC_H
#include FT_INTERNAL_DEBUG_H
#include FT_TRIGONOMETRY_H

#include "sdfgen.h"

  /* used to initialize vectors */
  static
  const FT_Vector   zero_vector = { 0, 0 };

  /* the process for generating signed distance field from       */
  /* outlines is based on the thesis:                            */
  /* Chlumsky, Viktor.Shape Decomposition for Multi-channel      */
  /* Distance Fields.Master's thesis. Czech Technical University */
  /* in Prague, Faculty of InformationTechnology, 2015.          */
  /* link: https://github.com/Chlumsky/msdfgen                   */

  FT_EXPORT_DEF( FT_Error )
  Generate_SDF( FT_Library     library,
                FT_GlyphSlot   glyph,
                FT_Bitmap     *abitmap )
  {
    SDF_Shape  shape;
    FT_Error   error   = FT_Err_Ok;

    FT_UInt    width   = 0u;
    FT_UInt    height  = 0u;
    FT_BBox    cBox;

    FT_Int     x_shift = 0;
    FT_Int     y_shift = 0;

    FT_Int     x_pad   = 0;
    FT_Int     y_pad   = 0;


    if ( !library )
      return FT_THROW( Invalid_Library_Handle );

    if ( !glyph || !abitmap || !glyph->face )
      return FT_THROW( Invalid_Argument );

    /* compute the width and height and add padding */
    FT_Outline_Get_CBox( &glyph->outline, &cBox );

    width =  FT_ABS( ROUND_F26DOT6( cBox.xMax - cBox.xMin ) ) / 64;
    height = FT_ABS( ROUND_F26DOT6( cBox.yMax - cBox.yMin ) ) / 64;

    x_pad = width / 4;
    y_pad = height / 4;

    width += x_pad;
    height += y_pad;

    x_shift = glyph->bitmap_left - x_pad / 2;
    y_shift = glyph->bitmap_top - glyph->bitmap.rows - y_pad / 2;

    /* align the outlne to the grid */
    FT_Outline_Translate(&glyph->outline, -x_shift * 64, -y_shift * 64);

    SDF_Shape_Init( &shape );
    shape.memory = library->memory;

    /* decompose the outline and store in into the SDF_Shape struct */
    /* this provide an easier way to iterate through all the curves */
    error = SDF_Decompose_Outline( &glyph->outline, &shape );
    if ( error != FT_Err_Ok )
      goto Exit;

    /* now loop through all the pixels and determine the shortest   */
    /* distance from the pixel's position to the nearest edge       */
    {
      FT_Memory      memory       = library->memory;
      FT_UInt        i            = 0u;
      FT_UInt        j            = 0u;

      float*     temp_buffer  = NULL;
      float      max_udist    = 0.0f; /* used to normalize values   */


      FT_MEM_ALLOC( temp_buffer, width * height * sizeof( float ) );

      for ( j = 0; j < height; j++ )
      {
        for ( i = 0; i < width; i++ )
        {
          FT_ULong      index = j * width + i;

          SDF_Contour*  head;        /* used to iterate            */
          FT_26D6Vec    current_pos; /* current pixel position     */

          SDF_Signed_Distance  min_dist;
          min_dist.distance = INT_MAX;


          head           = shape.head;

          /* add 0.5 to calculate distance from center of pixel */
          current_pos.x  = INT_TO_F26DOT6( i );
          current_pos.y  = INT_TO_F26DOT6( height - j );

          while ( head )
          {
            SDF_Signed_Distance   dist;
            FT_Fixed              udist;


            error = get_min_conour( head, current_pos, &dist );
            if ( error != FT_Err_Ok )
            {
              FT_MEM_FREE( temp_buffer );
              goto Exit;
            }

            if ( dist.distance < min_dist.distance )
              min_dist = dist;

            head = head->next;
          }


          temp_buffer[index] = ( float )min_dist.distance / 64.0f;
          if ( max_udist < temp_buffer[index] ) max_udist = temp_buffer[index];
          temp_buffer[index] *= min_dist.sign;
        }
      }

      /* release the previous buffer */
      FT_Bitmap_Done( library, abitmap );

      abitmap->width       = width;
      abitmap->rows        = height;
      abitmap->pitch       = width * sizeof( float );
      abitmap->num_grays   = 256;
      abitmap->pixel_mode  = 0;

      /* normalize the values and put in the buffer */
      for ( i = 0; i < width * height; i++ )
      {
        temp_buffer[i] = ( temp_buffer[i] ) / max_udist;
        temp_buffer[i] += 1.0f;
        temp_buffer[i] /= 2.0f;
        temp_buffer[i] = 1.0f - temp_buffer[i];
      }

      abitmap->buffer = ( unsigned char* )temp_buffer;
    }

    Exit:
    FT_Outline_Translate(&glyph->outline, x_shift * 64, y_shift * 64);
    SDF_Shape_Done( &shape );
    return error;
  }


  /**************************************************************************
   *
   * functions.
   *
   */

  static
  const SDF_Edge  null_sdf_edge = { { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                    { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                    SDF_EDGE_TYPE_NONE, NULL };

  static
  const SDF_Contour null_sdf_contour = { { 0.0f, 0.0f }, 
                                         SDF_CONTOUR_ORIENTATION_NONE,
                                         NULL, 0u };

  static
  const SDF_Shape  null_sdf_shape     = { NULL, 0u, NULL };

  FT_LOCAL_DEF( void )
  SDF_Edge_Init( SDF_Edge  *edge )
  {
    if ( edge )
      *edge = null_sdf_edge;
  }

  FT_LOCAL_DEF( void )
  SDF_Contour_Init( SDF_Contour  *contour )
  {
    if ( contour )
      *contour = null_sdf_contour;
  }

  FT_LOCAL_DEF( void )
  SDF_Shape_Init( SDF_Shape  *shape )
  {
    if ( shape )
      *shape = null_sdf_shape;
  }

  FT_LOCAL_DEF( FT_Error )
  SDF_Shape_Done( SDF_Shape  *shape )
  {
    FT_Memory     memory;
    SDF_Contour*  contour_head;

    if ( !shape )
      return FT_THROW( Invalid_Argument );

    if ( !shape->memory )
      return FT_THROW( Invalid_Library_Handle );

    memory = shape->memory;
    contour_head = shape->head;

    while ( contour_head )
    {
      SDF_Contour*  c          = contour_head;
      SDF_Edge*     edge_head  = c->head;


      while ( edge_head )
      {
        SDF_Edge*  e  = edge_head;


        edge_head = edge_head->next;
        FT_MEM_FREE( e );
      }

      contour_head = contour_head->next;
      FT_MEM_FREE( c );
    }

    *shape = null_sdf_shape;

    return FT_Err_Ok;
  }
  
  /* functions for FT_Outline_Decompose */

  static int
  sdf_outline_move_to( const FT_Vector*  to,
                       void*             user )
  {
    SDF_Shape*    shape     = ( SDF_Shape* )user;
    SDF_Contour*  contour   = NULL;

    FT_Error      error     = FT_Err_Ok;
    FT_Memory     memory    = shape->memory;


    FT_MEM_QNEW( contour );
    if ( error != FT_Err_Ok )
      return error;

    SDF_Contour_Init( contour );

    contour->last_pos     = *to;

    /* add contour to the shape */
    if ( shape->head )
      contour->next = shape->head;

    shape->head           = contour;
    shape->num_contours  += 1u;

    return error;
  }

  static int
  sdf_outline_line_to( const FT_Vector*  to,
                       void*             user )
  {
    SDF_Shape*    shape     = ( SDF_Shape* )user;
    SDF_Contour*  contour   = shape->head;
    FT_Vector     endpoint  = *to;
    SDF_Edge*     edge      = NULL;

    FT_Memory   memory      = shape->memory;
    FT_Error    error       = FT_Err_Ok;


    if ( endpoint.x != contour->last_pos.x ||
         endpoint.y != contour->last_pos.y )
    {
      /* only add edge if current_pos != to */
      FT_MEM_QNEW( edge );
      if (error != FT_Err_Ok)
        return error;
      SDF_Edge_Init( edge );
      
      edge->start_pos     = contour->last_pos;
      edge->end_pos       = endpoint;
      edge->edge_type     = SDF_EDGE_TYPE_LINE;

      /* add the edge to contour */
      if ( contour->head )
        edge->next = contour->head;

      contour->head        = edge;
      contour->last_pos    = endpoint;
      contour->num_edges  += 1u;
    }

    return error;
  }

  static int
  sdf_outline_conic_to( const FT_Vector*  control1,
                        const FT_Vector*  to,
                        void*             user )
  {
    SDF_Shape*    shape     = ( SDF_Shape* )user;
    SDF_Contour*  contour   = shape->head;
    FT_Vector     endpoint  = *to;
    FT_Vector     control   = *control1;
    SDF_Edge*     edge      = NULL;

    FT_Memory     memory    = shape->memory;
    FT_Error      error     = FT_Err_Ok;


    /* in conic bezier, to and current_pos can be same */
    FT_MEM_QNEW( edge );
    if (error != FT_Err_Ok)
        return error;
    SDF_Edge_Init( edge );

    edge->start_pos        = contour->last_pos;
    edge->end_pos          = endpoint;
    edge->control_point_a  = control;
    edge->edge_type        = SDF_EDGE_TYPE_QUADRATIC_BEZIER;

    /* add the edge to contour */
    if ( contour->head )
      edge->next = contour->head;

    contour->head        = edge;
    contour->last_pos    = endpoint;
    contour->num_edges  += 1u;

    return error;
  }

  static int
  sdf_outline_cubic_to( const FT_Vector*  control1,
                        const FT_Vector*  control2,
                        const FT_Vector*  to,
                        void*             user )
  {
    SDF_Shape*    shape      = ( SDF_Shape* )user;
    SDF_Contour*  contour    = shape->head;
    FT_Vector     endpoint   = *to;
    FT_Vector     control_a  = *control1;
    FT_Vector     control_b  = *control2;
    SDF_Edge*     edge       = NULL;

    FT_Memory     memory     = shape->memory;
    FT_Error      error      = FT_Err_Ok;


    /* in cubic bezier, to and current_pos can be same */
    
    FT_MEM_QNEW( edge );
    if (error != FT_Err_Ok)
        return error;
    SDF_Edge_Init( edge );

    edge->start_pos        = contour->last_pos;
    edge->end_pos          = endpoint;
    edge->control_point_a  = control_a;
    edge->control_point_b  = control_b;
    edge->edge_type        = SDF_EDGE_TYPE_CUBIC_BEZIER;

    /* add the edge to contour */
    if ( contour->head )
      edge->next = contour->head;

    contour->head       = edge;
    contour->last_pos   = endpoint;
    contour->num_edges  += 1u;

    return error;
  }

  FT_LOCAL_DEF( FT_Error )
  SDF_Decompose_Outline( FT_Outline*  outline,
                         SDF_Shape   *shape )
  {
    FT_Int        i       = 0;
    FT_Error      error   = FT_Err_Ok;

    SDF_Contour*  contour = NULL;


    /* initialize the FT_Outline_Funcs struct */
    FT_Outline_Funcs outline_decompost_funcs;
    outline_decompost_funcs.shift     = 0;
    outline_decompost_funcs.delta     = 0l;
    outline_decompost_funcs.move_to   = sdf_outline_move_to;
    outline_decompost_funcs.line_to   = sdf_outline_line_to;
    outline_decompost_funcs.conic_to  = sdf_outline_conic_to;
    outline_decompost_funcs.cubic_to  = sdf_outline_cubic_to;


    error=  FT_Outline_Decompose( outline, 
                                  &outline_decompost_funcs,
                                  ( void * )shape );
    if ( error != FT_Err_Ok )
      return error;

    contour = shape->head;
    while ( contour )
    {
      contour->orientation = get_contour_orientation( contour );
      contour = contour->next;
    }

    return error;
  }

  /**************************************************************************
   *
   * Math functions
   *
   */

  FT_LOCAL( SDF_Contour_Orientation )
  get_contour_orientation( SDF_Contour*  contour )
  {
    SDF_Edge*     head;

    FT_Fixed      area = 0;


    if ( !contour || !contour->head )
      return SDF_CONTOUR_ORIENTATION_NONE;

    head = contour->head;

    while ( head )
    {
      switch ( head->edge_type ) {
      case SDF_EDGE_TYPE_LINE:
      {
        area += ( head->end_pos.x - head->start_pos.x ) *
                ( head->end_pos.y + head->start_pos.y );
        break;
      }
      case SDF_EDGE_TYPE_QUADRATIC_BEZIER:
      {
        area += ( head->control_point_a.x - head->start_pos.x ) *
                ( head->control_point_a.y + head->start_pos.y );
        area += ( head->end_pos.x - head->control_point_a.x ) *
                ( head->end_pos.y + head->control_point_a.y );
        break;
      }
      case SDF_EDGE_TYPE_CUBIC_BEZIER:
      {
        area += ( head->control_point_a.x - head->start_pos.x ) *
                ( head->control_point_a.y + head->start_pos.y );
        area += ( head->control_point_b.x - head->control_point_a.x ) *
                ( head->control_point_b.y + head->control_point_a.y );
        area += ( head->end_pos.x - head->control_point_b.x ) *
                ( head->end_pos.y + head->control_point_b.y );
        break;
      }
      default:
          return SDF_CONTOUR_ORIENTATION_NONE;
      }

      head = head->next;
    }

    if ( area > 0.0f )
      return SDF_CONTOUR_ORIENTATION_CLOCKWISE;
    else
      return SDF_CONTOUR_ORIENTATION_ANTI_CLOCKWISE;
  }

  FT_LOCAL_DEF( FT_Error )
  get_min_conour( SDF_Contour*          contour,
                  const FT_26D6Vec      point,
                  SDF_Signed_Distance  *out )
  {
    SDF_Edge*             head;
    SDF_Signed_Distance   min_dist;

    FT_Error              error  = FT_Err_Ok;


    head                    = contour->head;
    min_dist.distance       = INT_MAX;

    /* currently we use a brute force algorithm to compute */
    /* shortest distance from all the curves               */
    while ( head )
    {
      SDF_Signed_Distance  dist;
    
    
      error = get_min_distance( head, point, &dist );
      if ( error != FT_Err_Ok )
        return error;
    
      if ( dist.distance < min_dist.distance )
      {
        min_dist = dist;
      }
      
      head = head->next;
    }

    *out = min_dist;

    return error;
  }

  FT_LOCAL_DEF( FT_Error )
  get_min_distance( SDF_Edge*             edge,
                    const FT_26D6Vec      point,
                    SDF_Signed_Distance  *out )
  {
    /* compute shortest distance from `point' to the `edge' */

    FT_Error  error = FT_Err_Ok;


    if ( !edge || !out )
      return FT_THROW( Invalid_Argument );

    switch ( edge->edge_type ) {
    case SDF_EDGE_TYPE_LINE:
    {
      /* in order to calculate the shortest distance from a point to */
      /* a line segment.                                             */
      /*                                                             */
      /* a = start point of the line segment                         */
      /* b = end point of the line segment                           */
      /* p = point from which shortest distance is to be calculated  */
      /* ----------------------------------------------------------- */
      /* => we first write the parametric equation of the line       */
      /*    point_on_line = a + ( b - a ) * t ( t is the factor )    */
      /*                                                             */
      /* => next we find the projection of point p on the line. the  */
      /*    projection will be perpendicular to the line, that is    */
      /*    why we can find it by making the dot product zero.       */
      /*    ( point_on_line - a ) . ( p - point_on_line ) = 0        */
      /*                                                             */
      /*                 ( point_on_line )                           */
      /*    ( a ) x-------o----------------x ( b )                   */
      /*                |_|                                          */
      /*                  |                                          */
      /*                  |                                          */
      /*                ( p )                                        */
      /*                                                             */
      /* => by simplifying the above equation we get the factor of   */
      /*    point_on_line such that                                  */
      /*                                                             */
      /* => we clamp the factor t between [0.0f, 1.0f], because the  */
      /*    point_on_line can be outside the line segment.           */
      /*                                                             */
      /*                                        ( point_on_line )    */
      /*    ( a ) x------------------------x ( b ) -----o---         */
      /*                                              |_|            */
      /*                                                |            */
      /*                                                |            */
      /*                                              ( p )          */
      /*                                                             */
      /* => finally the distance becomes | point_on_line - p |       */

      const FT_Vector   a                      = edge->start_pos;
      const FT_Vector   b                      = edge->end_pos;
      const FT_Vector   p                      = point;
      FT_Vector         line_segment           = zero_vector;
      FT_Vector         p_sub_a                = zero_vector;

      FT_Vector         nearest_point          = zero_vector;

      FT_Fixed          factor                 = 0;
      FT_Fixed          line_length            = 0;
      FT_Fixed          cross                  = 0;


      p_sub_a.x = p.x - a.x;
      p_sub_a.y = p.y - a.y;

      line_segment.x = b.x - a.x;
      line_segment.y = b.y - a.y;

      line_length = FT_Vector_Length( &line_segment );
      line_length = FT_MulFix( line_length, line_length );

      factor  = FT_MulDiv( p_sub_a.x, line_segment.x, line_length );
      factor += FT_MulDiv( p_sub_a.y, line_segment.y, line_length );

      /* clamp the factor between 0.0 and 1.0 in fixed point */
      if ( factor > ( 1 << 16 ) )
        factor = ( 1 << 16 );
      if ( factor < 0 )
        factor = 0;

      /* now `factor' is a purely fractional number so multiplication */
      /* with fixed point won't cause overflow                        */
      line_segment.x = FT_MulFix( line_segment.x, factor );
      line_segment.y = FT_MulFix( line_segment.y, factor );

      nearest_point.x = a.x + line_segment.x - p.x;
      nearest_point.y = a.y + line_segment.y - p.y;

      out->distance = FT_Vector_Length( &nearest_point );

      cross = FT_MulFix( nearest_point.x, b.y - a.y ) -
              FT_MulFix( nearest_point.y, b.x - a.x );

      out->sign = cross < 0 ? 1 : -1;

      break;
    }
    default:
      error = FT_THROW( Invalid_Argument );
    }

    return error;
  }

/* END */
