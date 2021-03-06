/* DOSFS basic directories example
 *
 * This example shows how to list files and subdirectories on a DOSFS filesystem
 *    
 * This example code is in the public domain.
 */

#include <DOSFS.h>

File myFile;

void setup ( void )
{
    Serial.begin(9600);

    while (!Serial) { }

    DOSFS.begin();

    myFile = DOSFS.open("a.txt", "w");
    myFile.close();

    myFile = DOSFS.open("b.txt", "w");
    myFile.close();

    myFile = DOSFS.open("c.txt", "w");
    myFile.close();

    DOSFS.mkdir("d");

    myFile = DOSFS.open("d/e0.txt", "w");
    myFile.close();

    myFile = DOSFS.open("d/e1.txt", "w");
    myFile.close();

    DOSFS.mkdir("d/e2");

    myFile = DOSFS.open("d/e2/f00.txt", "w");
    myFile.close();

    myFile = DOSFS.open("g.txt", "w");
    myFile.close();

    String root = String("/");
    
    printDir(root, 0);

    Serial.println("Done.");
}

void loop( void )
{
}

void printDir(const String &path, int ident)
{
    String filename, subdirectory;
    size_t size;
    bool isDirectory;
    Dir dir;

    dir = DOSFS.openDir(path);

    if (dir)
    {
        while (dir.read(filename, size, isDirectory))
        {
            for (int i = 0; i < ident; i++)
            {
                Serial.print(" ");
            }

            Serial.print(filename);

            if (isDirectory)
            {
                Serial.println();
          
                if ((filename != ".") && (filename != ".."))
                {
                    subdirectory = path;

                    if (path != "/") {
                        subdirectory += "/";
                    }
          
                    subdirectory += filename;
    
                    printDir(subdirectory, ident+2);
                }
            }
            else
            {
                Serial.print(", ");
                Serial.print(size);
                Serial.println();
            }
        }
    }
}
