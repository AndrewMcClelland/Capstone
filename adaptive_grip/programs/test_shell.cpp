#include "Shell.h"

using namespace std;

void test_one(arg_list list);
void test_two(arg_list list);

int main(int argc, char * argv[])
{
   Shell    shell;

   SHELL_ADD_FUNCTION(shell, test_one, "one", "test one")
   SHELL_ADD_FUNCTION(shell, test_two, "two", "test two")

   shell.run_shell();

   return   0;
}

void test_one(arg_list list){
   cout << "test_one() called." << endl;
}
void test_two(arg_list list){
   cout << "test_two() called." << endl;
}
