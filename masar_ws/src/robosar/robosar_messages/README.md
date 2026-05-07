# robosar_messages
ROS messages and agent-server comm protocol

### Building proto messages

* python ./../robosar_khepera_code/nanopb/generator/nanopb_generator.py -v -D proto/include/c proto/robosar.proto
* protoc robosar.proto --cpp_out=include/cpp
