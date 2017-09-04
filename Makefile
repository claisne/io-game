CPP_SHARED := -std=c++11 -O3 -I src src/main.cpp -luWS -lssl -lz
CPP_OPENSSL_OSX := -L/usr/local/opt/openssl/lib -I/usr/local/opt/openssl/include
CPP_OSX := -stdlib=libc++ -mmacosx-version-min=10.7 -undefined dynamic_lookup $(CPP_OPENSSL_OSX)

default:
	make `(uname -s)`
Linux:
	$(CXX) $(CPPFLAGS) $(CFLAGS) $(CPP_SHARED) -o io-game
Darwin:
	$(CXX) $(CPPFLAGS) $(CFLAGS) $(CPP_SHARED) $(CPP_OSX) -o io-game
