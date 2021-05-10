
#ifndef libLaneFreePlugin_EXPORT_H
#define libLaneFreePlugin_EXPORT_H

#ifdef libLaneFreePlugin_BUILT_AS_STATIC
#  define libLaneFreePlugin_EXPORT
#  define LIBLANEFREEPLUGIN_NO_EXPORT
#else
#  ifndef libLaneFreePlugin_EXPORT
#    ifdef libLaneFreePlugin_EXPORTS
        /* We are building this library */
#      define libLaneFreePlugin_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define libLaneFreePlugin_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef LIBLANEFREEPLUGIN_NO_EXPORT
#    define LIBLANEFREEPLUGIN_NO_EXPORT 
#  endif
#endif

#ifndef LIBLANEFREEPLUGIN_DEPRECATED
#  define LIBLANEFREEPLUGIN_DEPRECATED __declspec(deprecated)
#endif

#ifndef LIBLANEFREEPLUGIN_DEPRECATED_EXPORT
#  define LIBLANEFREEPLUGIN_DEPRECATED_EXPORT libLaneFreePlugin_EXPORT LIBLANEFREEPLUGIN_DEPRECATED
#endif

#ifndef LIBLANEFREEPLUGIN_DEPRECATED_NO_EXPORT
#  define LIBLANEFREEPLUGIN_DEPRECATED_NO_EXPORT LIBLANEFREEPLUGIN_NO_EXPORT LIBLANEFREEPLUGIN_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef LIBLANEFREEPLUGIN_NO_DEPRECATED
#    define LIBLANEFREEPLUGIN_NO_DEPRECATED
#  endif
#endif

#endif /* libLaneFreePlugin_EXPORT_H */
