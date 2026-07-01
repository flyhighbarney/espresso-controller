# Espresso Controller — Top-level Makefile

.PHONY: all firmware tests dashboard sysid results clean

all: firmware tests

# --- Firmware (ARM cross-compile) ---
firmware:
	cd firmware && mkdir -p build && cd build && \
		cmake .. && cmake --build .

# --- Host-side unit tests (native compile) ---
tests:
	cd firmware/Tests && mkdir -p build && cd build && \
		cmake .. && cmake --build . && ctest --output-on-failure

# --- Dashboard ---
dashboard:
	cd host/dashboard && python dashboard.py --simulate

dashboard-serial:
	cd host/dashboard && python dashboard.py --port $(PORT)

# --- System identification ---
sysid:
	cd host/sysid && python system_identification.py

sysid-chirp:
	cd host/sysid && python system_identification.py --generate-chirp

# --- ML training ---
train:
	cd host/training && python train_resistance_model.py --synthetic --epochs 200

train-export:
	cd host/training && python train_resistance_model.py \
		--synthetic --epochs 200 \
		--export-tflite model.tflite \
		--export-c-array ../../firmware/App/ml/model_data.h

# --- Generate results plots from logged data ---
results:
	cd host/sysid && python system_identification.py --data ../../data/step_response.csv
	@echo "Results plots saved to docs/"

# --- Clean ---
clean:
	rm -rf firmware/build firmware/Tests/build
	rm -f host/training/temp_model.tflite
