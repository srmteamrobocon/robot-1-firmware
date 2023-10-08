#include <iostream>

using namespace std;

class Node
{
public:
    int data;
    Node *next;

    Node(int val)
    {
        data = val;
        next = nullptr;
    }
};

Node *top = nullptr;

void push(int val)
{
    Node *newNode = new Node(val);
    if (!newNode)
    {
        cout << "Stack Overflow" << endl;
        return;
    }

    if (!top)
    {
        top = newNode;
    }
    else
    {
        newNode->next = top;
        top = newNode;
    }
}

void pop()
{
    if (!top)
    {
        cout << "Stack Underflow" << endl;
        return;
    }

    Node *temp = top;
    top = top->next;
    cout << "The popped element is " << temp->data << endl;
    delete temp;
}

void display()
{
    if (!top)
    {
        cout << "Stack is empty" << endl;
        return;
    }

    cout << "Stack elements are: ";
    Node *current = top;
    while (current)
    {
        cout << current->data << " ";
        current = current->next;
    }
    cout << endl;
}

int main()
{
    int ch, val;

    cout << "1) Push in the stack" << endl;
    cout << "2) Pop from the stack" << endl;
    cout << "3) Display the stack" << endl;
    cout << "4) Exit Program" << endl;

    do
    {
        cout << "Enter your choice: " << endl;
        cin >> ch;

        switch (ch)
        {
        case 1:
        {
            cout << "Enter value :" << endl;
            cin >> val;
            push(val);
            break;
        }
        case 2:
        {
            pop();
            break;
        }
        case 3:
        {
            display();
            break;
        }
        case 4:
        {
            cout << "Exit" << endl;
            break;
        }
        default:
        {
            cout << "Invalid Choice" << endl;
        }
        }
    } while (ch != 4);

    return 0;
}