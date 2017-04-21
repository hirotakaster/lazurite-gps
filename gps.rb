#! /usr/bin/ruby
require 'LazGem'
require 'mqtt'

laz = LazGem::Device.new
client = MQTT::Client.connect('mqtt://www.hogehoge.com')

finish_flag=0
Signal.trap(:INT){
	finish_flag=1
}
laz.init()
laz.begin(36,0xABCD,50,20)
print(sprintf("myAddress=0x%04x\n",laz.getMyAddress()))
laz.rxEnable()

# printing header of receiving log
print(sprintf("time\t\t\t\trxPanid\trxAddr\ttxAddr\trssi\tpayload\n"))
print(sprintf("------------------------------------------------------------------------------------------\n"))

# main routine
while finish_flag == 0 do
    if laz.available() <= 0
        next
    end

    rcv = laz.read()
    if rcv["payload"].length >= 9 then
        print(sprintf("rx_time= %s\trx_nsec=%d\trssi=%d\t",Time.at(rcv["sec"]),rcv["nsec"],rcv["rssi"]));
        seq = rcv["payload"][0].ord;
        lat = rcv["payload"][1..4].unpack("i*")[0]/1000000.to_f;
        lon = rcv["payload"][5..8].unpack("i*")[0]/1000000.to_f;
        print(seq);
        print("\t");
        print(lat);
        print("\t");
        print(lon);
        print("\n");
        client.publish('920M/', seq.to_s + "," + lat.to_s + "," + lon.to_s, retain=false)
        STDOUT.flush
    end
end

laz.remove();


