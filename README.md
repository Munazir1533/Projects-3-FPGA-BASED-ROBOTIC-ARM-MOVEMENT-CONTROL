# Projects-3-FPGA-BASED-ROBOTIC-ARM-MOVEMENT-CONTROL
                          `ABSTRACT`
In this era of fast-paced technological advancements, automation has become an integral part of almost every 
industry, from manufacturing to healthcare. When we think of automation, we often envision robots, which 
have revolutionized the manufacturing sector. Among the most widely used robots in manufacturing 
applications is the Robotic Arm, which can perform tasks like assembly line operations, packaging and 
similar operations.
There are two main methods of controlling a robotic arm to perform specific tasks: using a microcontroller or 
designing a custom ASIC package. While a microcontroller-based approach offers programmability, it can be 
slow due to memory read/write operations required to execute instructions. Conversely, a custom ASIC 
package can deliver better and faster performance, but it is expensive for small scale production and can take 
longer time to build. Since an FPGA offers programmability and a quick prototype design process, it is the 
preferred technology for low-volume applications and can be used to implement the robotic arm controller.
In our project, we adopted the FPGA methodology and used the Xilinx Artix-7 series FPGA on the 
Nexys4DDR board. SG90 servomotors were used to control the robotic arm, and Verilog HDL was used to 
program the FPGA. We implemented our design using FPGA and simulation using the Xilinx Vivado tool. 
An operational two-DOF FPGA-based robotic arm controller was developed by our team. The suggested arm 
is designed to automatically follow a specific trajectory

                `Verilog HDL Code used in the Projects`
`1. Top Module:`

module Sawtooth_Gen(
 input clk,
 output reg [20:0] counter=0
 );
always @(posedge clk) 
begin
 
 if (counter < 'd1999999) 
 counter <= counter+1;
 else begin
 counter <= 0;
 end
end
endmodule

`2. Saw-tooth Generator Module: `

module Sawtooth_Gen(
 input clk,
 output reg [20:0] counter=0
 );
always @(posedge clk) 
begin
 
 if (counter < 'd1999999) 
 counter <= counter+1;
 else begin
 counter <= 0;
 end
end
endmodule

3. PWM Generator
module PWM_Generator(
input [16:0]ref_1,
input [17:0] ref_2,
input [16:0] ref_3,
input [20:0]sawtooth,
output pwm1,pwm2,pwm3
 );

 4. Address Generator Module
    module Addr_Gen(
 input enable,reset,
 input [20:0] sawtooth,
 output reg [8:0] addr='d485,
 output reg[16:0] ref3='d0
 );
 
 always @(negedge sawtooth[20])
begin
 if(enable) begin
 if(addr=='d485) 
 addr <= 0; 
 else begin
 addr <= addr + 1;
 ref3 <= 0;
 end 
 if(addr>='d436) 
 ref3 <= 'd51440;
 else if(addr == 0 )
 ref3 <= 0; 
 end
 if(reset) 
 begin
 if(addr>0)
 addr <= addr-1;
 else addr<=0;
 
 ref3 <= 'd51440;
end
end
endmodule

5. Testbench
   module PWM_tb;
reg clk=0;
reg En=1;
reg reset=0;
wire PWMout1;
wire PWMout2;
wire PWMout3;
PWM2 DUT(clk,En,reset,PWMout1,PWMout2,PWMout3);
always #5 clk = ~clk;
always begin
#50000000 En=0;
#30000000 En=1;
#100000000 En=0;
#150000000 reset = !reset;
endmodule

![sim](https://github.com/Munazir1533/Projects-3-FPGA-BASED-ROBOTIC-ARM-MOVEMENT-CONTROL/assets/93303360/483a9453-f988-4627-87ae-34fff9176be0)

      `MATLAB CODE`
% Trajectory Generation
L = 0.11; b=0.3; a = L*b; w=0.02;
for i = 1:length(x)-1
 if((x(i)-w)<L && (x(i)-w)>0 )
 y1(i) = sqrt(L^2 - (x(i)-w-(L/2)).^2);
 end
end
for i = 1:length(x)
 if((x(i)-w)<L && (x(i)-w)>0 )
 y2(i) = sqrt(L^2 - (x(i)-w-(L/2)).^2) + b*(x(i)-w)-a;
 end
end
for i = 1:length(x)
 if((x(i)-w)<L && (x(i)-w)>0 )
 y3(i) = sqrt(L^2 - ((x(i))-w-(L/2)).^2)-a;
 end
end
for i = 1:length(x)
 if((x(i)-w)<L && (x(i)-w)>0 )
 y4(i) = sqrt(L^2 - (x(i)-w-(L/2)).^2) + b*(x(i)-w)-2*a;
 end
end
% Inverse Kinematics
Len1 = 0.087; Len2 = 0.084;
px = wp(1,:); py = wp(2,:);
c2 = (px.*px + py.*py -Len1*Len1-Len2*Len2)./(2*Len1*Len2);
s21 = sqrt(1-c2.*c2);
s22 = -sqrt(1-c2.*c2);
theta2 = atan2(s21,c2); 
denom = L1*L1 +L2*L2 + 2*L1*L2*cos(theta2);
s1 = (py.*(Len1+Len2*cos(theta2))-px.*(Len2*sin(theta2)));
c1 = (px.*(Len1+Len2*cos(theta2))+py.*(Len2*sin(theta2)));
theta1 = atan2(s1,c1);
q1 = 90+(180/pi)*theta1; q2 = (180/pi)*theta2;
pw1 = q1/90; pw2 = q2/90; 
counter1 = pw1 *1e5; counter2 = pw2 *1e5;
binary1 = string(fliplr(de2bi(round(counter1))));
binary2 = string(fliplr(de2bi(round(counter2))));
binstr1=[];
for i =1:size(binary1,2)
binstr1 = strcat(binstr1,binary1(:,i)); 
end
binstr2=[];
for i =1:size(binary2,2)
binstr2 = strcat(binstr2,binary2(:,i)); 
end



![SCHEMATIC](https://github.com/Munazir1533/Projects-3-FPGA-BASED-ROBOTIC-ARM-MOVEMENT-CONTROL/assets/93303360/9041d576-587a-4dcc-aee0-ba71c4eea2a9)



