#include "StdAfx.h"
#include "export.h"
#include "stdio.h"


void trace(char *s, ...)
{
#ifdef COMPILE_ALCHEMY
	AS3_Trace(AS3_String(s));
#endif
}