set tdesc filename target.xml

target remote localhost:3333
set remotetimeout 30
set print pretty on

define flashwrite
  monitor flash write_image erase $arg0
end

define reset_halt
  monitor reset halt
end

define reset_run
  monitor reset halt
  continue
end
