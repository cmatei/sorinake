

slaves = {}

function Slave(args)
   if not args.name then
      error("Slave declaration missing name attribute.")
   end

   local s = {}

   s.bus = args.bus or 1
   s.address = args.address or 0x32

   args.calibration = args.calibration or {}
   s.calibration = args.calibration

   s.calibration[1] = args.calibration[1] or { a = 1, b = 0 }
   s.calibration[2] = args.calibration[2] or { a = 1, b = 0 }
   s.calibration[3] = args.calibration[3] or { a = 1, b = 0 }
   s.calibration[4] = args.calibration[4] or { a = 1, b = 0 }

   -- allocate _data
   s._data = slave_data_alloc(s)

   -- set metatable to access _data
   setmetatable(s, {
		   __index = slave_data_index,
		   __newindex = slave_data_newindex,
		})

   slaves[args.name] = s
end
