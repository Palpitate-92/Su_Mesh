#include <iostream>
#include <vector>
using namespace std;

class Book
{
public:
    char bookName[10], bookAuthor[10], bookSno[5], bookPress[20]; //定义字符数组
    float money;
    Book(char[], char[], char[], char[], float);                  //定义含参构造函数
    void print();                                                 //定义打印方法
    ~Book();                                                      //定义析构函数
};

//为构造函数赋值
Book::Book(char name[], char author[], char sno[], char press[], float money)
{
    strcpy_s(bookName, name); //字符数组不可以直接赋值给另一个字符数组，要用strcpy_s函数将右边的字符串或字符数组复制给左边的字符数组。
    strcpy_s(bookAuthor, author);
    strcpy_s(bookSno, sno);
    strcpy_s(bookPress, press);
    this->money = money; //变量与父类重名时使用this调用父类的变量，注意this→变量名不是this.变量名
}

void Book::print()
{
    cout << "书名：" << bookName << "\n作者：" << bookAuthor << "\n书号：" << bookSno << "\n出版社：" << bookPress << "\n价格：" << money;
}

Book::~Book()
{
    cout << "\n执行完毕！";
}

int main()
{
    char name[] = "C++", author[] = "摸鱼", sno[] = "1234", press[] = "摸鱼王";
    Book book(name, author, sno, press, 20.21);
    book.print();
}
