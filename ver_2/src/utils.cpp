#include "..\include\utils.h"

/*bool CheckContinue(){
    getchar();
    string userInput;
    while(true){
        cout << "Continue? (Y/n): ";
        getline(cin, userInput);
        if(userInput == "" || userInput == "y"){
            return true;
        }
        if(userInput == "n") {
            return false;
            break;
        }
    }
}
*/

bool CheckContinue(){
    std::string userInput;
    while(true){
        std::cout << "Continue? (Y/n): ";
        std::getline(std::cin, userInput);
        if(userInput == "" || userInput == "y" || userInput == "Y"){
            return true;
        }
        if(userInput == "n" || userInput == "N") {
            return false;
        }
    }
}