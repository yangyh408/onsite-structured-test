import TessNG

def main():
    mode = 'CREATE_TESS'
    config = {'tasks': None, 'dt': 0.1, 'maxTestTime': 30, 'skipExist': True}
    TessNG.run(mode, config, None, auto_run=False)

if __name__ == '__main__':
    main()