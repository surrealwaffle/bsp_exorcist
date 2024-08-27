#ifndef HLEF_BASE_H
#define HLEF_BASE_H

// TODO (cmake): Use generate_export_header instead!

#ifdef HLEF_EXPORT
# define HLEF_API  __declspec(dllexport)
# define HLEF_CALL __cdecl
#else
# define HLEF_API  __declspec(dllimport)
# define HLEF_CALL __cdecl
#endif

#endif // HLEF_BASE_H