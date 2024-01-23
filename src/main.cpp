#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "cpu/mst.hpp"
#include "iograph.hpp"

#define __PROGRAM_VERSION__ "v1.0.0"

constexpr char INPUT_GRAPH_PATH[] = "./graph.txt";
constexpr char OUTPUT_GRAPH_PATH[] = "./mst.txt";
constexpr char STREAM[] = "no";
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
        { "--graph", INPUT_GRAPH_PATH },
        { "--mst", OUTPUT_GRAPH_PATH },
        { "--device", DEVICE },
        { "--stream", STREAM }
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
        } else if (key == "--stream") {
            if (!(value == "no" || value == "yes")) {
                throw std::runtime_error("Invalid device option: " + value + ". Choose one of ['no', 'yes'].");
            }
        }

        options[key] = value;
    }

    return options;
}

int main(int32_t argc, char const* argv[])
{
    auto start = std::chrono::high_resolution_clock::now();

    try {
        auto options = parse_arguments(argc, argv);

        ReadGraph reader {};
        WriteGraph writer {};
        InGraph in_graph {};
        OutGraph out_graph {};

        if (options["--stream"] == "no") {
            std::cout << "Steiner tree problem.\n";
            std::cout << "Program version: " << __PROGRAM_VERSION__ << '\n';
            std::cout << "C++ Standard: " << __cplusplus << "\n\n";
            std::cout << "Program options:\n";
            std::cout << "[--graph] Path to the input graph file: " << options["--graph"] << '\n';
            std::cout << "[--mst] Path to the output mst file: " << options["--mst"] << '\n';
            std::cout << "[--device] Device to use: " << options["--device"] << "\n";
            std::cout << "[--stream] Stream mode: " << options["--stream"] << "\n\n";
            std::cout << std::flush;

            in_graph = reader(options["--graph"]);

            if (options["--device"] == "cpu") {
                CpuMST mst {};
                out_graph = mst(in_graph);
            } else {
            }

            auto results = writer(out_graph, options["--mst"]);

            std::cout << "Total MST nodes: " << results["total_nodes"] << '\n';
            std::cout << "Total MST edges: " << results["total_edges"] << '\n';
            std::cout << "Total MST weight: " << results["total_weight"] << '\n';

            std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start;
            std::cout << "\nExecution Time: " << diff.count() << " s\n";
            std::cout << std::flush;
        } else {
            std::string stream_token;

            while (true) {
                std::ostringstream iss {};

                while (std::getline(std::cin, stream_token)) {
                    if (stream_token == "EXIT") {
                        std::cout << "Exit successfully." << std::endl;
                        return EXIT_SUCCESS;
                    }

                    if (stream_token == "EOF") {
                        break;
                    }

                    iss << stream_token << '\n';
                }

                std::istringstream oss(iss.str());
                in_graph = reader(oss);

                if (options["--device"] == "cpu") {
                    CpuMST mst {};
                    out_graph = mst(in_graph);
                }

                std::cout << writer(out_graph) << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
