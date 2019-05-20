a.out: *.c src/*.c src/*.h apue/src/libapue.a
	gcc -Werror -Wextra -I src -I apue/src -L apue/src src/horizon_link.c transceiver.c -lapue
