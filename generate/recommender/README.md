# Text capability recommendation generator

## Purpose

Generate human-readable recommendations on avenues of exploration of generally-phrased problems.

Example: "I want to build a better mousetrap"

## Ideas

Can use GPT-{2,3,...} or another text synthesis program trained on e.g. Hackaday articles to draw from prior maker ingenuity. 

UI would be a text prompt, followed by a list of generated paragraphs streamed in parallel from a GPU-run instance of a GPT model. User can "refresh" if they don't like the results, or select a result to add it to the training set.

The end delivered goal is not so much a fully coherent plan, but more of an "idea generator" of a much higher quality than exists today (e.g. https://ideagenerator.creativitygames.net/). 

Will still require some human effort to turn the suggestions into anything useful. 
