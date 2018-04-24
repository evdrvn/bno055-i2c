all: bno055-i2c-example 

bno055-i2c-example: ../src/bno055-i2c.c src/main.c
	gcc -Wall -O2 \
		-I../src -Iext/civetweb/include -Iext/evdsptc/src \
		-Lext/evdsptc/build  -Lext/civetweb \
		../src/bno055-i2c.c src/main.c \
		-rdynamic -Wl,-dn,-levdsptc,-lcivetweb,-dy,-lm,-ldl,-lpthread -o bno055-i2c-example 

clean:
	rm -f bno055-i2c-example
