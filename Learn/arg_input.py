import argparse

def main():
    argparser = argparse.ArgumentParser(description = "Demo Parser")
    argparser.add_argument('--host', default='127.0,0,1')
    argparser.add_argument('--port', type = int, default = 2000)
    argparser.add_argument('--verbose', action = 'store_true')
    args = argparser.parse_args()   

    if args.verbose:
        print("Verbose mode is ON")
    print(f"Connecting to {args.host}: {args.port}")

if __name__ == '__main__':
    main()