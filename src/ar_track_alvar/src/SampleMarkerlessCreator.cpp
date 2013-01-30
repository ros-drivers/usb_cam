#include "FernImageDetector.h"
using namespace std;
using namespace alvar;

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleMarkerlessCreator" << std::endl;
        std::cout << "=======================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'FernImageDetector' class" << std::endl;
        std::cout << "  to train a Fern classifier for markerless image-based tracking." << std::endl;
        std::cout << "  The image should contain many unique features and be in the range" << std::endl;
        std::cout << "  of 200x200 to 500x500 pixels. A '.dat' file will be saved in the" << std::endl;
        std::cout << "  same directory as the image and can be used with the" << std::endl;
        std::cout << "  SampleMarkerlessDetector sample." << std::endl;
        std::cout << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << filename << " filename" << std::endl;
        std::cout << std::endl;
        std::cout << "    filename filename of image to train" << std::endl;
        std::cout << std::endl;

        if (argc < 2) {
            std::cout << "Filename not specified." << std::endl;
            return 0;
        }

        std::cout << "Training classifier." << std::endl;
        FernImageDetector fernDetector(true);
        std::string imageFilename(argv[1]);
        fernDetector.train(imageFilename);

        std::cout << "Writing classifier." << std::endl;
        std::string classifierFilename = imageFilename + ".dat";
        if (!fernDetector.write(classifierFilename)) {
            std::cout << "Writing classifier failed." << std::endl;
            return 1;
        }

        return 0;
    }
    catch (const std::exception &e) {
        std::cout << "Exception: " << e.what() << endl;
    }
    catch (...) {
        std::cout << "Exception: unknown" << std::endl;
    }
}
