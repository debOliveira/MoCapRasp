import logging
import multiprocessing
import time
from dataclasses import dataclass
from multiprocessing import Process
from multiprocessing.managers import BaseManager
from threading import Thread

import pandas
import warnings
warnings.filterwarnings('ignore')


@dataclass
class DataFrameHolder:
    """This dataclass holds a reference to the current DF in memory.
    This is necessary if you do operations without in-place modification of
    the DataFrame, since you will need replace the whole object.
    """
    dataframe: pandas.DataFrame = pandas.DataFrame(columns=['A', 'B'])

    def update(self, data):
        self.dataframe = self.dataframe.append(data, ignore_index=True)

    def retrieve(self):
        return self.dataframe


class DataFrameManager(BaseManager):
    """This dataclass handles shared DataFrameHolder.
    See https://docs.python.org/3/library/multiprocessing.html#examples
    """
    # You can also use a socket file '/tmp/shared_df'
    MANAGER_ADDRESS = ('localhost', 6000)
    MANAGER_AUTH = b"auth"

    def __init__(self) -> None:
        super().__init__(self.MANAGER_ADDRESS, self.MANAGER_AUTH)
        self.dataframe: pandas.DataFrame = pandas.DataFrame(columns=['A', 'B'])

    @classmethod
    def register_dataframe(cls):
        BaseManager.register("DataFrameHolder", DataFrameHolder)


class DFWorker:
    """Abstract class initializing a worker depending on a DataFrameHolder."""

    def __init__(self, df_holder: DataFrameHolder) -> None:
        super().__init__()
        self.df_holder = df_holder


class StreamLoader(DFWorker):
    """This class is our worker communicating with the websocket"""

    def update_df(self):
        # read from websocket and update your DF.
        data = {
            'A': [1, 2, 3],
            'B': [4, 5, 6],
        }
        self.df_holder.update(data)

    def run(self):
        # limit loop for the showcase
        for _ in range(4):
            self.update_df()
            print("[1] Updated DF\n%s" % str(self.df_holder.retrieve()))
            time.sleep(3)


class LightComputation(DFWorker):
    """This class is a random computation worker"""

    def compute(self):
        print("[2] Current DF\n%s" % str(self.df_holder.retrieve()))

    def run(self):
        # limit loop for the showcase
        for _ in range(4):
            self.compute()
            time.sleep(5)


def main():
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)

    # Register our DataFrameHolder type in the DataFrameManager.
    DataFrameManager.register_dataframe()
    manager = DataFrameManager()
    manager.start()
    # We create a managed DataFrameHolder to keep our DataFrame available.
    df_holder = manager.DataFrameHolder()

    # We create and start our update worker
    stream = StreamLoader(df_holder)
    stream_process = Thread(target=stream.run)
    stream_process.start()

    # We create and start our computation worker
    compute = LightComputation(df_holder)
    compute_process = Process(target=compute.run)
    compute_process.start()

    # The managed dataframe is updated in every Thread/Process
    time.sleep(1)
    print("[0] Main process DF\n%s" % df_holder.retrieve())

    # We join our Threads, i.e. we wait for them to finish before continuing
    stream_process.join()
    compute_process.join()

main()