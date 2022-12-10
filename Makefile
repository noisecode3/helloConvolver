CC=g++ -g
IDIR=FFTConvolver
CFLAGS=-I$(IDIR) -march=native -ffast-math -fomit-frame-pointer

helloConvolver: Utilities.o AudioFFT.o FFTConvolver.o main.cpp
	$(CC) -o helloConvolver main.cpp AudioFFT.o FFTConvolver.o Utilities.o $(CFLAGS)

Utilities.o: $(IDIR)/Utilities.cpp $(IDIR)/Utilities.h
	$(CC) -c $(CFLAGS) $(IDIR)/Utilities.cpp

AudioFFT.o: $(IDIR)/AudioFFT.cpp $(IDIR)/AudioFFT.h
	$(CC) -c $(CFLAGS) $(IDIR)/AudioFFT.cpp

FFTConvolver.o: $(IDIR)/FFTConvolver.cpp $(IDIR)/FFTConvolver.h
	$(CC) -c $(CFLAGS) $(IDIR)/FFTConvolver.cpp


.PHONY: clean

clean:
	rm -f *.o helloConvolver out.wav
