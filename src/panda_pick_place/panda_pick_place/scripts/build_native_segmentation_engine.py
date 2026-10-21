"""Build 424x240 PeopleSemSegNet IO with internal 448x256 padding and crop.

Preserves model weights and default TensorRT precision; saves a separate engine.
Run this script directly with --onnx SOURCE --output NEW_ENGINE.
"""

import argparse
from pathlib import Path
import tensorrt as trt

args = argparse.ArgumentParser()
args.add_argument('--onnx', required=True)
args.add_argument('--output', required=True)
opt = args.parse_args()
height, width = 240, 424
padded_height, padded_width = 256, 448
logger = trt.Logger(trt.Logger.INFO)
builder = trt.Builder(logger)
network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
parser = trt.OnnxParser(network, logger)
if not parser.parse(Path(opt.onnx).read_bytes()):
    raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
if network.num_inputs != 1 or network.num_outputs != 1:
    raise RuntimeError('Expected single-input/single-output PeopleSemSegNet')
source = network.get_input(0)
output = network.get_output(0)
print('Original IO:', source.shape, output.shape, flush=True)
consumers = [(network.get_layer(i), j) for i in range(network.num_layers)
             for j in range(network.get_layer(i).num_inputs)
             if network.get_layer(i).get_input(j) == source]
if not consumers:
    raise RuntimeError('Model input has no consumers')
source.shape = (1, 3, height, width)
if (padded_height, padded_width) != (height, width):
    padding = network.add_padding_nd(source, (0, 0),
                                     (padded_height - height, padded_width - width))
    padding.name = 'native_input_padding_to_stride32'
    for layer, index in consumers:
        layer.set_input(index, padding.get_output(0))
shape = tuple(output.shape)
print('Padded output:', shape, flush=True)
if shape == (1, padded_height, padded_width, 1):
    cropped_shape = (1, height, width, 1)
elif shape == (1, padded_height, padded_width):
    cropped_shape = (1, height, width)
else:
    raise RuntimeError('Unexpected padded mask shape: ' + str(shape))
name = output.name
network.unmark_output(output)
output.name = name + '_padded'
crop = network.add_slice(output, (0,) * len(shape), cropped_shape, (1,) * len(shape))
crop.name = 'native_output_crop'
crop.get_output(0).name = name
network.mark_output(crop.get_output(0))
config = builder.create_builder_config()
config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 2 << 30)
engine = builder.build_serialized_network(network, config)
if engine is None:
    raise RuntimeError('Engine build failed')
Path(opt.output).write_bytes(engine)
print('SAVED', opt.output, flush=True)
