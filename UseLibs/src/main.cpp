#include "main.h"

int parseInputArgs(int argc, char **argv, std::string& directory,
        std::string& image, int& k, Controller::KCPP_MODE& mod);

int main(int argc, char **argv) 
{
    std::string image;
    std::string directory;
    int k = 1;
    Controller::KCPP_MODE mod;

    if(parseInputArgs(argc, argv, directory, image, k, mod) == -1) {
        return EXIT_FAILURE;
    }

    Controller controller;
    /*NOTE: Just checking*/
    try {
        controller.run(directory, image, k,mod);
    } 
    catch (std::invalid_argument ia) {
        std::cerr << ia.what() << "\n";
    }
    return 0;

};

int parseInputArgs(int argc, char **argv, std::string& directory,
        std::string& image, int& k, Controller::KCPP_MODE& mod)
{
    bool printUsage = false;
    // Parse and validate command line input arguments
    if (argc <= 4) 
    {
        printUsage = true;
    }

    else 
    {
        try 
        {
            directory = boost::lexical_cast<string>(argv[1]) + "/";
            image = boost::lexical_cast<string>(argv[2]);
            k = boost::lexical_cast<int>(argv[3]);
            int tmp = boost::lexical_cast<int>(argv[4]);
            mod = static_cast<Controller::KCPP_MODE>(tmp);

            if (directory.size() <= 0) 
            {
                printUsage = true; 
            }
            else if (image.size() <= 0) 
            {
                printUsage = true;
            }
        }

        catch (std::exception err)
        {
            printUsage = true;
        }
    }


    if (printUsage)
    {
        std::cerr << "Usage: " << argv[0] << " [IMAGE_DIRECTORY] [IMAGE_NAME]"
            << " [NUMBER_OF_ROBOTS]" << "[KCPP_ALGORITHM_MODE \n -> 0 FHK algorithm \n -> 1 CAC algorithm]" << std::endl;
        return -1;
    }

    return 0;
}


