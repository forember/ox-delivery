#include "main.h"

/*****************************************************************************
 * @author: Kelly Benson
 * @date: May 31, 2015
 * @email: bensonke@email.sc.edu
 *
 * Main function for the world load program 
 *
**/


int parseInputArgs(int argc, char **argv, std::string& directory,
        std::string& image, int& k);

int main(int argc, char **argv) 
{
    std::string image;
    std::string directory;
    int k = 1;

    if(parseInputArgs(argc, argv, directory, image, k) == -1) {
        return EXIT_FAILURE;
    }

    Controller controller;
    /*NOTE: Just checking*/
    try {
        controller.run(directory, image, k);
    } 
    catch (std::invalid_argument ia) {
        std::cerr << ia.what() << "\n";
    }
    return 0;

};

int parseInputArgs(int argc, char **argv, std::string& directory,
        std::string& image, int& k)
{
    bool printUsage = false;
    // Parse and validate command line input arguments
    if (argc <= 3) 
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
            << " [NUMBER_OF_ROBOTS]" << std::endl;
        return -1;
    }

    return 0;
}


