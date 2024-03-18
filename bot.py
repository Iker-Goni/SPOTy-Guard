import discord
import asyncio

from discord import Intents

# Define your bot's intents
intents = Intents.default()
intents.messages = True  # You may need to adjust these based on your bot's functionality

# Initialize the Discord client with intents
client = discord.Client(intents=intents)
# Token goes here

# Define your boolean condition and the message you want to send
boolean_condition = True
message_to_send = "BARK! I can speak!"

# Define an event that triggers when the bot is ready
@client.event
async def on_ready():
    print(f'{client.user} has connected to Discord!')
    # Start the task once the bot is ready
    await send_message_if_true()

# Define a function to send the message when the boolean condition is true
async def send_message_if_true():
    while not client.is_closed():
        if boolean_condition:
            channel = client.get_channel(1219100542752657510)  # Replace with your channel ID
            await channel.send(message_to_send)
        await asyncio.sleep(60)  # Check every minute

# Start the bot
client.run(TOKEN)
