idf_component_register(SRCS "src/predictor.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES espressif__esp-tflite-micro esp-tflite-micro
                    )