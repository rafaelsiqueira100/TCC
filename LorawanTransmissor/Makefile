SHELL := powershell.exe
.SHELLFLAGS := -NoProfile -ExecutionPolicy Bypass -Command

IDF_PATH := C:/esp/v6.0/esp-idf
IDF_TARGET := esp32
PROJECT_DIR := C:/Users/rafae/workspace/LorawanTransmissor

all:
	& { \
		Set-Location '$(IDF_PATH)'; \
		.\install.ps1 esp32; \
		. .\export.ps1; \
		$$env:IDF_TARGET = '$(IDF_TARGET)'; \
		Set-Location '$(PROJECT_DIR)'; \
		Set-Location C:/Users/rafae/workspace/LorawanTransmissor; \
		Remove-Item -Recurse -Force build/;\
		Remove-Item -Force sdkconfig;\
		python '$(IDF_PATH)/tools/idf.py' set-target esp32;\
		python '$(IDF_PATH)/tools/idf.py' --cmake-warn-uninitialized set-target esp32; \
		python '$(IDF_PATH)/tools/idf.py' build; \
	}