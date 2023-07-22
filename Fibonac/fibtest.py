

def fib(n:int)->int:
    return n if n <= 1 else fib(n - 1) + fib(n - 2)

if __name__ == "__main__":
    for x in range(10):
        print(f'Fibonacii {x} is {fib(x)}')
