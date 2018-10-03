#include <Arduino>


char target[] = {'1', '3', '5'};
char num_target = 3;

char check_target(char scan_char)
{
    for (char id = 0; id < num_target; ++id)
    {
        if (target[id] == scan_char)
            return id+1;
    }
    return 0;
}

