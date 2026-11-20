# third_party

`path_receiver_node` needs two single-header libraries in this folder:

| File | Library | Get it |
|------|---------|--------|
| `httplib.h` | [cpp-httplib](https://github.com/yhirose/cpp-httplib) (MIT) | `curl -LO https://raw.githubusercontent.com/yhirose/cpp-httplib/master/httplib.h` |
| `json.hpp` | [nlohmann/json](https://github.com/nlohmann/json) (MIT) | `curl -LO https://github.com/nlohmann/json/releases/latest/download/json.hpp` |

Both are MIT-licensed, so committing them here is fine and makes the package build out of the box.
