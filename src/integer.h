/*-------------------------------------------*/

#ifndef A038F6A3_40DC_4F8A_9318_A3CAE07B18FA
#define A038F6A3_40DC_4F8A_9318_A3CAE07B18FA
/* Integer type definitions for FatFs module */
/*-------------------------------------------*/

#ifndef _FF_INTEGER
#define _FF_INTEGER

#ifdef _WIN32	/* FatFs development platform */

#include <windows.h>
#include <tchar.h>

#else			/* Embedded platform */

/* This type MUST be 8 bit */
typedef unsigned char	BYTE;

/* These types MUST be 16 bit */
typedef short			SHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;

/* These types MUST be 16 bit or 32 bit */
typedef int				INT;
typedef unsigned int	UINT;

/* These types MUST be 32 bit */
typedef long			LONG;
typedef unsigned long	DWORD;

#endif

#endif


#endif /* A038F6A3_40DC_4F8A_9318_A3CAE07B18FA */
