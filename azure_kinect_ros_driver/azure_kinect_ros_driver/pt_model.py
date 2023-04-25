from torch import nn
import torch

# Here we define our model as a class
class LSTM(nn.Module):

    def __init__(self, input_size, hidden_size, out_size=1,
                 num_layers=1):
        super(LSTM, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers

        # Define the LSTM layer
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers)
        self.dropout = nn.Dropout(p=0.5)
        # Define the output layer
        self.linear = nn.Linear(hidden_size, out_size)

    def init_hidden(self):
        # This is what we'll initialise our hidden state as
        return (torch.zeros(self.num_layers, self.hidden_size),
                torch.zeros(self.num_layers, self.hidden_size))

    def forward(self, sequence, hidden):
        # Forward pass through LSTM layer
        # shape of lstm_out: [input_size, batch_size, hidden_dim]
        # shape of self.hidden: (a, b), where a and b both
        # have shape (num_layers, batch_size, hidden_dim).
        lstm_out, hidden = self.lstm(sequence, hidden)
        # Only take the output from the final timestep
        # Can pass on the entirety of lstm_out to the next layer if it is a seq2seq prediction
        #x = lstm_out[-1].view(self.batch_size, -1)
        x = self.dropout(lstm_out)
        y_pred = self.linear(x)
        return y_pred, hidden