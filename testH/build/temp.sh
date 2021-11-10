
g++ -o $1 "../$1.cpp" `pkg-config --cflags --libs opencv4`
