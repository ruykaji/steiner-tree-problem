#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

constexpr char GRAPH_PATH[] = "./graph.txt";
constexpr char DEVICE[] = "cpu";

static inline std::unordered_map<std::string, std::string> parse_arguments(int32_t argc, char const* argv[])
{
    auto get_value = [argc, argv](int32_t t_expected, const std::string& t_key) {
        if (t_expected >= argc) {
            throw std::runtime_error("Missing value for argument: " + t_key);
        }
        return std::string(argv[t_expected]);
    };

    std::unordered_map<std::string, std::string> options {
        { "--graph", GRAPH_PATH },
        { "--device", DEVICE }
    };

    for (int i = 1; i < argc; i += 2) {
        std::string key = argv[i];

        if (options.find(key) == options.end()) {
            throw std::runtime_error("Unknown argument: " + key);
        }

        std::string value = get_value(i + 1, key);

        if (key == "--graph") {
            if (!std::filesystem::exists(value)) {
                throw std::runtime_error("Can not find graph with path: " + value);
            }
        } else if (key == "--device") {
            if (!(value == "cpu" || value == "gpu")) {
                throw std::runtime_error("Invalid device option: " + value + ". Choose one of ['cpu', 'gpu'].");
            }
        }

        options[key] = value;
    }

    std::cout << std::flush;

    return options;
}

int main(int32_t argc, char const* argv[])
{
    try {
        auto options = parse_arguments(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return 0;
}
