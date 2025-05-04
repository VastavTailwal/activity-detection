#include "model.h"
#include <stdio.h>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"


#define INPUT_SIZE 16
#define OUTPUT_SIZE 7
#define TENSOR_ARENA_SIZE (50 * 1024)


static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* input = nullptr;
static TfLiteTensor* output = nullptr;
static uint8_t tensor_arena[TENSOR_ARENA_SIZE];


extern "C" int predict(const float* input_data)
{
    static bool initialized = false;

    if (!initialized)
    {
        const tflite::Model* model = tflite::GetModel(ActiveNetV2_tflite);
        if (model == nullptr || model->subgraphs() == nullptr)
        {
            printf("Invalid model!\n");
            return -1;
        }

        // Declare supported ops
        static tflite::MicroMutableOpResolver<5> resolver;

        resolver.AddSoftmax();
        resolver.AddFullyConnected();
        resolver.AddReshape();
        resolver.AddQuantize();
        resolver.AddDequantize();

        // Construct interpreter without error reporter
        static tflite::MicroInterpreter static_interpreter(
            model,
            resolver,
            tensor_arena,
            TENSOR_ARENA_SIZE
            // Remaining args use defaults (resource vars, profiler, use_allocated_arena)
        );

        interpreter = &static_interpreter;

        if (interpreter->AllocateTensors() != kTfLiteOk)
        {
            printf("Failed to allocate tensors!\n");
            return -1;
        }

        input = interpreter->input(0);
        output = interpreter->output(0);

        initialized = true;
    }

    // Copy input data
    for (int i = 0; i < INPUT_SIZE; ++i)
        input->data.f[i] = input_data[i];

    // Run inference
    if (interpreter->Invoke() != kTfLiteOk)
    {
        printf("Model invoke failed!\n");
        return -1;
    }

    int prediction = -1;
    float maximum = output->data.f[0];

    for (int i = 1; i < OUTPUT_SIZE; ++i)
    {
        if (output->data.f[i] > maximum)
        {
            maximum = output->data.f[i];
            prediction = i;
        }
    }

    return prediction;
}