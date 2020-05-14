
#include <ft2build.h>
#include FT_INTERNAL_DEBUG_H

#include FT_FREETYPE_H
#include FT_ERRORS_H
#include FT_INTERNAL_OBJECTS_H

#include "sdfgen.h"

  FT_EXPORT_DEF( FT_Error )
  FT_Generate_SDF( FT_Library   library,
                   FT_Outline*  outline, 
                   FT_Bitmap   *abitmap )
  {
    
    
    
    return FT_THROW( Unimplemented_Feature );
  }


  /**************************************************************************
   *
   * functions.
   *
   */

  static
  const SDF_Contour null_sdf_contour = { { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                         { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                         SDF_CONTOUR_TYPE_NONE, NULL };

  static
  const SDF_Shape null_sdf_shape = { { 0.0f, 0.0f }, NULL, 0u };

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
  SDF_Shape_Done( FT_Library  library,
                  SDF_Shape  *shape )
  {
    FT_Memory memory;


    if ( !library )
      return FT_THROW( Invalid_Library_Handle) ;

    if ( !shape )
      return FT_THROW( Invalid_Argument );

    memory = library->memory;
    
    SDF_Contour * head = shape->contour_head;

    while ( head )
    {
      SDF_Contour * temp = head;
      head = head->next;

      FT_FREE( temp );
    }

    *shape = null_sdf_shape;

    return FT_Err_Ok;
  }
  

/* END */
