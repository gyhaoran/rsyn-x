#ifndef C976A209_FEB3_46AA_93C8_C3654B74DC77
#define C976A209_FEB3_46AA_93C8_C3654B74DC77

////////////////////////////////////////////////////////////////////////////////
#define __DO_STRINGIZE( symbol ) #symbol
#define STRINGIZE(symbol) __DO_STRINGIZE(symbol)

////////////////////////////////////////////////////////////////////////////////
#define __DO_JOIN_AGAIN(symbol1, symbol2) symbol1##symbol2
#define __DO_JOIN(symbol1, symbol2) __DO_JOIN_AGAIN(symbol1, symbol2)

////////////////////////////////////////////////////////////////////////////////
#define JOIN(symbol1, symbol2) __DO_JOIN(symbol1, symbol2)

////////////////////////////////////////////////////////////////////////////////
#ifdef __COUNTER__
# define UNIQUE_ID __COUNTER__
#else
# define UNIQUE_ID __LINE__
#endif

////////////////////////////////////////////////////////////////////////////////
# define UNIQUE_NAME(prefix) JOIN(prefix, UNIQUE_ID)

#endif /* C976A209_FEB3_46AA_93C8_C3654B74DC77 */