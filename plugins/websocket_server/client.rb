#!/usr/bin/env ruby


require 'faye/websocket'
require 'eventmachine'

threads = []
for i in 0..958
  threads << Thread.new {
    EM.run {
      ws = Faye::WebSocket::Client.new('ws://localhost:9002')

      ws.on :open do |event|
        p [:open]
        ws.send('sub,/world/simple_cave_01/dynamic_pose/info,,')
      end

      ws.on :message do |event|
        p [:message, i]
      end

      ws.on :close do |event|
        p [:close, event.code, event.reason]
        ws = nil
      end
    }
  }
end

threads.each { |thr| thr.join }
