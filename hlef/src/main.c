#include <windows.h>

#include "hlef_context.h"

BOOL APIENTRY DllMain(HINSTANCE hInstance, DWORD fdwReason, LPVOID lpvReserved)
{
  int error = 0; // 0 indicates no error, non-zero if error
  switch (fdwReason) {
    case DLL_PROCESS_ATTACH: error = hlef_load(); break;
    case DLL_PROCESS_DETACH: hlef_unload();       break;
    default:                                      break;
  }
  return error ? FALSE : TRUE;
}