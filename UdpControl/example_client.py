"""SDK example. Status by default; --move explicitly enables actual motors."""
import argparse
import json
import math
import time

from .client import MotorClient


def main():
    parser=argparse.ArgumentParser(description='SV635N UDP Python 客户端示例')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=5005)
    parser.add_argument('--auth-key',default='')
    parser.add_argument('--move',action='store_true',help='实际开启电机并执行运动；默认仅查询状态')
    parser.add_argument('--adapter',help='专用 EtherCAT 网卡完整名称；省略使用后台配置')
    parser.add_argument('--orders',nargs='+',type=int,default=[1,2,3])
    parser.add_argument('--rpm',type=float,default=60)
    parser.add_argument('--acceleration',type=float,default=120)
    parser.add_argument('--amplitude',type=float,default=5)
    parser.add_argument('--period',type=float,default=10)
    parser.add_argument('--seconds',type=float,default=20)
    args=parser.parse_args()
    if not all(math.isfinite(x) for x in (args.amplitude,args.period,args.seconds)) or args.period<=0 or args.seconds<=0:
        parser.error('幅度须有限，周期和时长须为正的有限数值。')
    with MotorClient(args.host,args.port,args.auth_key) as client:
        if not args.move:
            print(json.dumps(client.status(),ensure_ascii=False,indent=2))
            return
        client.hello()
        client.request('scan',**({'adapter':args.adapter} if args.adapter else {}))
        client.wait_for(('idle',))
        client.enable(args.orders,args.rpm,args.acceleration)
        client.wait_for(('enabled',))
        started=time.monotonic()
        try:
            while time.monotonic()-started<args.seconds:
                value=args.amplitude*math.sin((time.monotonic()-started)*2*math.pi/args.period)
                reply=client.target([value]*len(args.orders))
                if reply['state']['phase']!='enabled':
                    raise RuntimeError(reply['state']['message'])
                time.sleep(1/60)
        finally:
            client.disable()
            state=client.wait_for(('idle','fault'))
            print(json.dumps(state,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
