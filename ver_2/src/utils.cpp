#include "..\include\utils.h"

bool CheckContinue(){
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
