import tensorrt as trt
import numpy as np
import ctypes
import os

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def load_engine(engine_file_path):
    """Load a TensorRT engine from a file."""
    with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def allocate_buffers(engine):
    """Allocate host and device buffers for TensorRT inference."""
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()

    for binding in engine:
        binding_shape = engine.get_binding_shape(binding)
        binding_size = trt.volume(binding_shape) * engine.max_batch_size
        dtype = trt.nptype(engine.get_binding_dtype(binding))

        # Allocate host and device buffers
        host_mem = cuda.pagelocked_empty(binding_size, dtype)
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        bindings.append(int(device_mem))

        # Assign input/output buffers
        if engine.binding_is_input(binding):
            inputs.append((host_mem, device_mem))
        else:
            outputs.append((host_mem, device_mem))

    return inputs, outputs, bindings, stream

def do_inference(context, bindings, inputs, outputs, stream):
    """Run inference using TensorRT."""
    # Transfer input data to the GPU
    [cuda.memcpy_htod_async(inp[1], inp[0], stream) for inp in inputs]

    # Execute the inference
    context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)

    # Transfer predictions back to the CPU
    [cuda.memcpy_dtoh_async(out[0], out[1], stream) for out in outputs]

    # Synchronize the stream
    stream.synchronize()

    # Return the outputs
    return [out[0] for out in outputs]

# Path to your engine file
engine_path = "nuScenesBest.engine"

# Load the TensorRT engine
engine = load_engine(engine_path)
context = engine.create_execution_context()

# Allocate buffers
inputs, outputs, bindings, stream = allocate_buffers(engine)

# Prepare a dummy input for inference (adjust shape as per your model)
input_data = np.random.randn(1, 3, 640, 640).astype(np.float32)
np.copyto(inputs[0][0], input_data.ravel())

# Run inference
output = do_inference(context, bindings, inputs, outputs, stream)
print("Inference output:", output)
