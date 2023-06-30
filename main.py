from task import BBI_Trainer
from config import OPTIONS as config

def main(config):
    Task = BBI_Trainer(config)
    Task.start_session()

if __name__ == "__main__":
    main(config)