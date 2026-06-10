###############################################################################
# Makefile for project 
###############################################################################

## General options
PROJECT = 
MCU = Arduino Mega
COMPILER = "Arduino AVR (Proteus)"
TARGET = Debug
SHELL = C:\Windows\system32\cmd.exe

## Tools general options
APPFLAGS=-C mega2560 -T -std=gnu++11 -T -fno-threadsafe-statics -N atmega2560 -F 16000000 -U "C:/ProgramData/Labcenter Electronics/Proteus 9 Professional/VSM Studio/drivers/Arduino"
CCFLAGS=-Wall -gdwarf-2 -fno-exceptions -ffunction-sections -fdata-sections -DF_CPU=16000000 -I .. -I "C:/ProgramData/Labcenter Electronics/Proteus 9 Professional/VSM Studio/drivers/Arduino" -DARDUINO_ARCH_AVR @arduino/libs.inc -std=gnu++11 -fno-threadsafe-statics -O0 -mmcu=atmega2560
LDFLAGS=-Wl,--gc-sections -lm -mmcu=atmega2560

## Processing Tools
FLOWCHART=
APP=mksketch
CC=avr-gcc
ASM=avr-gcc
LD=avr-gcc
#  G:/내 드라이브/경기과학기술대학교/2026년1학기/지능형임베디드시스템실습1/2026 제16회 전국대학교 IT융합 및 전자회로설계 경진대회/platformio/IT2026_platformio/proteus9/ARDUINO_MEGA_1/Debug/Debug.elf: TOOL=LD
#  Post: avr-objcopy -O ihex -R .eeprom "./Debug.elf" "./Debug.hex"
#  Post: avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 --no-change-warnings -O ihex "./Debug.elf" "./Debug.eep" || exit 0 
#     G:/내 드라이브/경기과학기술대학교/2026년1학기/지능형임베디드시스템실습1/2026 제16회 전국대학교 IT융합 및 전자회로설계 경진대회/platformio/IT2026_platformio/proteus9/ARDUINO_MEGA_1/Debug/main.o: TOOL=CC
#        G:/내 드라이브/경기과학기술대학교/2026년1학기/지능형임베디드시스템실습1/2026 제16회 전국대학교 IT융합 및 전자회로설계 경진대회/platformio/IT2026_platformio/proteus9/ARDUINO_MEGA_1/Debug/main.cpp: TOOL=APP
#        Post: make -f arduino/Makefile all
#           G:/내 드라이브/경기과학기술대학교/2026년1학기/지능형임베디드시스템실습1/2026 제16회 전국대학교 IT융합 및 전자회로설계 경진대회/platformio/IT2026_platformio/proteus9/ARDUINO_MEGA_1/main.ino

# Build tree:
all:	Debug

Debug:	Debug.elf
	avr-size "Debug.ELF"

Debug.elf:	main.o 
	$(LD) $(LDFLAGS)  -o "./Debug.elf" "main.o" arduino/arduino.a -lm
	avr-objcopy -O ihex -R .eeprom "./Debug.elf" "./Debug.hex"
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 --no-change-warnings -O ihex "./Debug.elf" "./Debug.eep" || exit 0 

main.o:	main.cpp 
	$(CC) $(CCFLAGS)  -o "main.o" -c "main.cpp"

main.cpp:	../main.ino 
	$(APP) $(APPFLAGS)  -A "C:/Program Files/Labcenter Electronics/Proteus 9 Professional/Tools/Arduino" -O main.cpp "../main.ino"
	make -f arduino/Makefile all

# bootloader - attach bootloader to the firmware
bootloader:
	avr-objcopy -O ihex -R .eeprom "./Debug.elf" "./Debug.hex"
	copy /b bootloader.hex+"./Debug.hex" "with.bootloader.hex"

# tidy - delete all temporary files which are not involved in the target generation
tidy:
	rm -rf ./main.d
	-make -f arduino/Makefile tidy

# cleanup - delete all generated files
clean:	tidy
	rm -rf Debug.elf
	rm -rf main.o
	rm -rf main.cpp
	-make -f arduino/Makefile clean
