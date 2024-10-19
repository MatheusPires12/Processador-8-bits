module pc(
    input clk,
    input zero,
    input [9:0] comandos, // Sinais de controle
    input [15:0] instruction, // Instrução de 16 bits
    output reg [15:0] address // Endereço de 16 bits
);

    // Variaveis intermediarias
    wire [6:0] immediate = instruction [6:0]; // Campo imediato pra I-TYPE
    wire [12:0] instr_index = instruction [12:0]; // Indice para J-TYPE
    wire [15:0] ext_immediate = {9'd0, immediate}; // Estende o imediato para 16 bits
    wire [15:0] shift = ext_immediate << 2; // Desloca o imediato para branch
    wire muxBranch = comandos[3]; // Sinal de controle de Branch
    wire muxJump = comandos[0]; // Sinal de controle de Jump

    // Iniciando o endereço PC
    initial begin
        address = -16'd2; // Inicializa com -2 para compensar o incremento inicial
    end

    // Funcionamento do PC
    always @(posedge clk) begin
        address <= address + 16'd2; // Incrementa o PC de 2 em 2 (instruções de 16 bits)

        // Condição do Jump (J-TYPE)
        if (muxJump == 1'b1) begin
            address <= {address[15:13], instr_index << 2}; //Salta para o indice da instrução
        end

        // Condição do Branch (I-TYPE)
        if ((muxBranch == 1'b1) && (zero == 1'b1) && (muxJump == 1'b0)) begin
            address <= address + shift; // Desvio condicional (branch)
        end
    end
endmodule

// Memória de instruções
module instructions_memory(
    input clk,
    input [15:0] pc, //Contador de programa
    output [15:0] instruction // Instrução de 16 bits
);

    integer f, r;   // Arquivo e retorno de leitura
    reg [7:0] data = 8'd0; // Para armazenar temporariamenteos dados lidos do arquivo
    integer counterMemory = 16'd0;

    // Criando um memória de instruções com 100 endereços de 8 bits
    reg [7:0] memoriaInstrucoes [99:0]; // Cada instrução ocupa 2 bytes (16 bits)

    // Inicialização de memória de instruções
    initial begin
        // Abrindo o arquivo com as instruções (assume-se que as instruções estão em formato binário)
        f = $open("/content/instructions.txt", "r");

        // Se o arquivo não puder ser aberto
        if (f == NULL) begin
            $display("Arquivo sem instruções");
            $finish;
        end

        // Lendo os dados do arquivo
        while (!feof(f)) begin
            r = $fscanf(f, "%8b", data); // Lê o byte (8 bits) por vez
            memoriaInstrucoes[counterMemory] = data; //Armazena o byte na memória
        end

        // fechando o arquivo
        $fclose(f);
    end 

    // Atribuindo o valor da instrução de 16 bits
    assign instruction = { memoriaInstrucoes[pc], memoriaInstrucoes[pc + 1]}; // Primeiro e segundo byte da instrução
endmodule

// Banco de registradores
module register_bank (
    input clk,
    input [9:0] comandos,
    input [15:0] instruction, // Instrução de 16 bits
    input [15:0] data_in, // Entrada de dados de 16 bits
    output [15:0] out1, // Saídas de 16 bits
    output [15:0] out2,
    output [15:0] out3,
    output reg [127:0] memory // Para armazenar 8 registradores de 16 bits (128 bits no total)
);

    //Variaveis intermediárias
    wire RegDst = comandos[9];
    wire ALUscr = comandos[8];
    wire RegWrite = comandos[6];

    // Campos de endereços conforme o formato da isntrução
    wire [2:0] addr_regd, addr_reg1, addr_reg2;
    assign addr_reg1 = instruction[12:10]; // rs
    assign addr_reg2 = instruction[9:7]; // rt
    assign addr_regd =  (RegDst == 1'b0) ? instruction[9:7]: instruction[6:4]; // rd ou rt dependendo de RegDst

    // Extensão do imediato (7 bits para o imediato)
    wire [15:0] extensao = {9'd0, instruction[6:0]}; // Extensão do imediato para 16 bits

    // Criando um banco de registradores com 8 endereços de 16 bits
    reg [15:0] registerBank [7:0];

    // Inicializando o banco de registradores
    integer i;
    initial begin
        for (i = 0; i < 8; i = i + 1) begin
            registerBank[i] <= 16'd0;
        end
    end

    // Descrevendo o comportamento de escrita no banco de registradores
    always @(posedge clk) begin
        if (RegWrite == 1'b1) begin
            registerBank[addr_regd] <= data_in;
        end
    end

    // Atribuindo os valores de saída
    assign out1 = registerBank[addr_reg1]; // Valor do registrador res
    assign out2 = (ALUscr == 1'b1) ? extensao : registerBank[addr_reg2]; // Valor do registrador rt ou imediato
    assign out3 = registerBank[addr_reg2]; // Sempre retorna o valor de rt

    // Atualizando a memória visível (primeiros 4 registradores)
    always @(posedge clk) begin
        memory <= {registerBank[0], registerBank[1], registerBank[2], registerBank[3]};
    end
endmodule

// Unidade logica aritmetica (ULA)
module ula (
    input [2:0] control_ula, // Controle da ULA com 3 bits
    input [15:0] in1, // Entradas de 16 bits
    input [15:0] in2, 
    output reg [15:0] out, // Saída com 16 bits
    output reg zero // Flag zero

);

    // Descrevendo o comportamento da ULA
    always @( in1, in2, control_ula) begin
        case (control_ula)
            3'b010: out = in1 + in2; // Soma
            3'b110: out = in1 - in2; // Subtração
            3'b000: out = in1 & in2; // AND
            3'b001: out = in1 | in2; // OR
            default: out = 16'd0; // Padrão: zero
        endcase

    // Definindo a flag zero se o resultado for 0
    if (out == 16'd0) begin
        zero = 1'b1;
    end
    else begin
        zero = 1'b0;
    end
endmodule

// Unidade de controle
module control (
    input [2:0] opcode, // Opcode de 3 bits 
    output reg [9:0] out // Sinais de controle
);
    always @(opcode) begin
        case (opcode)
            3'b000: out = 10'b1001000100; // Tipo R
            3'b000: out = 10'b0111100000; // LW (Load word - Tipo I)
            3'b000: out = 10'bx1x0010000; // SW (Store word - Tipo I)
            3'b000: out = 10'bx0x0001010; // BEQ (Branch on Equal - Tipo I)
            3'b000: out = 10'bxxx0x0xxx1; // Jump (Tipo J)
            default: out = 10'bxxxxxxxxxx; // Padrão
        endcase
    end
endmodule

// Controle da ULA
module control_ULA (
    input [1:0] ULAOp, //Sinais de controle da ULA
    input [3:0] funct, // Funct de 4 bits (bits [3:0] da  instrução)
    output reg [2:0] out // Saída de controle da ULA
);
    always @(ULAOp, funct) begin
        case({ULAOp, funct})
            6'b00xxxx: out 3'b010; // LW, SW (Tipi I) - Soma
            6'b01xxxx: out 3'b110; // BEQ (Tipo I) - Subtração
            6'b10_0000: out 3'b010; // Soma
            6'b10_0010: out 3'b110; // Subtração
            6'b10_0100: out 3'b000; // AND
            6'b10_0101: out 3'b001; // OR
            default: out 3'bxxx; // Padrão (Jump ou não definido)
        endcase
    end
endmodule

// Memória de dados
module data_memory (
    input clk,
    input MemWrite,
    input MemRead,
    input MemtoReg,
    input [15:0] addr, // Endereço de memória
    input [15:0] write_data, // Dado a ser escrito
    output reg [15:0] read_data, // Dado lido
    output reg [63:0] memory // Estado de uma parte da memória (4 palavras de 16 bits)
);

    // Construindo memória com 256 palavras de 16 bits
    reg [15:0] ram [0:255];

    // Inicializando a memória com valores iniciais
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            ram[i] = i;
        end
    end

    // Operações de leitura e escrita
    always @(posedge clk) begin
        case({MemWrite, MemRead, MemtoReg})
            3'b000: read_data <= addr; // Nenhuma operação (retorna endereço)
            3'b011: read_data <= ram[addr]; // Leitura de memória
            3'b10x: ram[addr] <= write_data; // Escrita de memória
        endcase
    end

    //Atualização de parte da memória para monitoramento
    always @(posedge clk) begin
        memory <= {ram[0], ram[1], ram[2], ram[3]}; // Monitorando os primeiros 4 registros da memória
    end
endmodule