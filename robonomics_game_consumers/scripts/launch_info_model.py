import datetime
from collections import Callable
from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship, sessionmaker
from sqlalchemy import create_engine

Base = declarative_base()

class Launch(Base):
    __tablename__ = 'Launch'
    id = Column(Integer, primary_key=True)
    timestamp = Column(DateTime, default=datetime.datetime.utcnow)
    timestamp_start = Column(DateTime)
    timestamp_finish = Column(DateTime)
    block_number_start = Column(Integer, nullable=False)
    block_number_finish = Column(Integer, nullable=False)

session = sessionmaker()
engine = create_engine('sqlite:///launch_info.db')
session.configure(bind=engine)
Base.metadata.create_all(engine)

def stamp_launch(ts_start, ts_finish, bn_start, bn_finish):
    s = session()
    if isinstance(ts_finish, Callable) and isinstance(bn_finish, Callable):
        launch = Launch(timestamp_start=ts_start, timestamp_finish=ts_finish(),
                        block_number_start=bn_start, block_number_finish=block_number_finish())
    else:
        launch = Launch(timestamp_start=ts_start, timestamp_finish=ts_finish,
                        block_number_start=bn_start, block_number_finish=block_number_finish)
    s.add(launch)
    s.commit()

def get_last_launch_num():
    s = session()
    last_launch_num = 0
    if s.query(Launch).first(): # table not empty
        last_launch = s.query(Launch).order_by(Launch.id.desc()).first()
        last_launch_num = last_launch.id
    return last_launch_num

def get_launch_blocks(launch_num):
    s = session()
    launch = Launch(block_number_start=0, block_number_finish=0)
    if s.query(Launch).filter(Launch.id == launch_num).count(): # if launch exists
        launch = s.query(Launch).filter(Launch.id == launch_num).one()
    return [launch.block_number_start, launch.block_number_finish]
