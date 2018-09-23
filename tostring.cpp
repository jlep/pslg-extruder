#include <iostream>
#include <fstream>

std::string basename(const std::string& filename) {
    auto dot = filename.rfind('.');
    auto slash = filename.rfind('/', dot);
    return filename.substr(slash + 1, dot - slash - 1);
}

int main(int argc, char** argv) {
    std::string out_filename = argv[1];
    std::ofstream out;
    out.open(out_filename);
    for (int i = 2; i < argc; ++i) {
        auto in_filename = std::string(argv[i]);
        std::ifstream in(in_filename);
        out << "const char* " << basename(in_filename) << "_shader_str = \"\"";
        std::string line;
        while (std::getline(in, line)) {
            out << "\n    \"" << line << "\\n\"";
        }
        out << ";\n";
    }
    return 0;
}
