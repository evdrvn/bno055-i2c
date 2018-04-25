all: bno055-i2c-example 

bno055-i2c-example: ../build/libbno055-i2c.so ext/civetweb/libcivetweb.so ext/evdsptc/build/libevdsptc.so src/main.c
	gcc -Wall -O2 \
		-I../src -Iext/civetweb/include -Iext/evdsptc/src \
		-L../build -Lext/evdsptc/build  -Lext/civetweb \
		src/main.c \
		-rdynamic -Wl,-dn,-lbno055-i2c,-levdsptc,-lcivetweb,-dy,-lm,-ldl,-lpthread -o bno055-i2c-example 

../build/libbno055-i2c.so:
	cd ../build; cmake ..; make 

ext/civetweb/libcivetweb.so:
	cd ext/civetweb; make lib WITH_WEBSOCKET=1

ext/evdsptc/build/libevdsptc.so:
	cd ext/evdsptc/build; cmake ..; make 

clean:
	rm -f bno055-i2c-example
