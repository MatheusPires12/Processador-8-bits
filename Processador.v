module data_memory (
    input clk, // Sinal de clock para sincronização da escrita
    input [15:0] mem_acess_addr, // Endereço de acesso à memória
    input [15:0] mem_write_data, // Dados a serem escritos na memória
    input mem_write_en,  // Habilita escrita
    input mem_read, // Habilita leitura
    output [15:0] mem_read_data // Dados lidos da memória
);

    integer i; // Variável para uso no loop de inicialização da memória

    reg [15:0] ram [255:0]; // Definição da memória RAM: 256 posições, cada uma com 16 bits

    wire [15:0] ram_addr = mem_acess_addr[8:1];// Calcula o endereço da memória com base nos bits [8:1] do endereço de entrada

    initial begin // Inicializa a memória, preenchendo todas as 256 posições com 0
        for(i = 0; i < 256; i++)
            ram[i] <= 16'd0; // Atribui valor 0 (16 bits) a cada posição de memória
    end

    always @(posedge clk) begin // Executado na borda de subida do clock
        if(mem_write_en) // Se habilitado, realiza a escrita
            ram[ram_addr] <= mem_write_data; // Escreve o dado na posição ram_addr
    end
    // Atribui o dado lido à saída mem_read_data, se mem_read estiver em 1, lê o dado da memória;
    // Caso contrário, retorna 0
    assign mem_read_data = (mem_read == 1'b1) ? ram[ram_addr] : 16'd0;

endmodule