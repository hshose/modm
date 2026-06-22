`ethernet_throughput` is a UDP payload benchmark for the `nucleo_h723zg` LAN8742A ethernet setup.

The board uses:
- control UDP port `5000`
- data UDP port `5001`
- static IP `192.168.10.50`

Host-side helper:
```bash
python3 udp_throughput.py upload --board-ip 192.168.10.50 --duration 5 --payload-size 1400
python3 udp_throughput.py download --board-ip 192.168.10.50 --duration 5 --payload-size 1400
```

`upload` measures host-to-board throughput.
`download` measures board-to-host throughput.
