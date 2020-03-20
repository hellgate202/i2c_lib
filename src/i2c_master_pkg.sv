package i2c_master_pkg;

parameter bit [2 : 0] NOP   = 3'b000;
parameter bit [2 : 0] START = 3'b001;
parameter bit [2 : 0] STOP  = 3'b010;
parameter bit [2 : 0] READ  = 3'b011;
parameter bit [2 : 0] WRITE = 3'b100;

endpackage
